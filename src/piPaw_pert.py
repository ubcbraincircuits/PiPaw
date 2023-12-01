"""
piPaw lever cage program for assessing forelimb motor learning in the mouse home-cage.
The main criteria for success in this version is the time the lever is held in a reward range.
This testing methodology consists of two training phases followed by an main phase:

    Training:       Lever must be pulled past the threshold position. There are no failed trials,
                    anything past the threshold is successful.
                    The trial ends when the lever returns below threshold.
                    Mouse must perform 200 successful trials in this phase in order to
                    move onto the Main phase
                    During this phase, the mouse also gets a free water drop every
                    15 minutes just for nose poking (FI Schedule)
    Main:           Lever must be pulled back between 6 and 18 degrees
                    (middle half of the full position range) and held for a certain amount of time
                    The amount of time the lever must be held in this range starts at 0.1 seconds,
                    and increases as the mouse learns to hold for longer
                    If the lever exits the goal range before the required time has passed,
                    the trial is failed
                    Once the mouse has performed n trials at each hold time (adjustable),
                    the program begins checking their success rate with a sliding window of
                    50 trials If they are over 75% successful in this window of trials,
                    they are advanced to a longer hold time (0.1s higher) to a maximum of 1s
                    There is no maximum hold time limit for the trial as long as the lever is
                    in the central goal range.
                    However, once the lever leaves the goal range (either above or below),
                    the trial is finished and the motor returns slowly to high torque to return
                    lever to start position.

For all testing phases, the mouse initiates trial by nose poking and
must leave nose in the port for the duration of the trial.
Mice proceed at their own pace through the task in a group housed environment.
Trial timing, outcome, lever position data and video is collected for all trials.
"""

import datetime
import gc
import multiprocessing as mp
import random

from sys import exit
from os import path, mkdir
from datetime import datetime, timedelta
from time import time, sleep, mktime, strptime

import RPi.GPIO as GPIO
import numpy as np
import picamera
import serial

from .encoder import Encoder
from .pwm import PWM


#GPIO Constants
G_INFRARED = 21

# Specific GPIO for this should be between 1-8 as these are pulled High by default on boot,
# otherwise valve will open
G_SOLENOID = 4

G_MOTOR_ENABLE = 23
G_MOTOR_DIRECTION = 24
G_RFID = 16

#Trial constants

# Number of successful trials in training stage
N_TRAINING = 200

# Size of the bin in which trials are checked to determine if animal advances to next hold time
N_SUCCESS_CHECK = 50

# Success ratio required for increase in hold time
SUCCESS_REQ = 0.75

# Initial hold time at beginning of main phase
INITIAL_HT = 0.05

# Amount hold time is increased by each time
HT_INCREASE = 0.1

# Maximum hold time mice can advance to
MAX_HT = 1.0

# Length of time solenoid opens for entrance rewards and successful trials
T_SOLENOID = 0.25

# Minimum duration between two trials
T_TIMEOUT = 5.0

# Minimum duration between two successful trials
T_TIMEOUT_SUCCESS = 1.0

# Minimum duration between two cancelled trials
T_TIMEOUT_CANCELLED = 5.0

# Minimum duration between failed trials
T_TIMEOUT_FAILED = 5.0

# This raises the gain a bit on the sensitivity of the IR breakbeam, as
# it sometimes goes high for a fraction of a second when a small amount of light gets through
T_IR_GRACE = 0.2

# Duration after last RFID signal detection that animal is considered to have left the chamber
T_RFID_GRACE = 30.0

# Minimum duration IR breakbeam must be broken for before trial initiates
# (even if timeout is finished)
T_IR_WAIT = 1.0

# Fixed interval for delivery of entrance reward in seconds (only for training 1 phase)
T_ENTRANCE_REWARD_INTERVAL = 900

# Time to wait before starting the perturbation
PERTURBATION_WAIT = 0.5

# % of max force for perturbation
PERT_LIST = [-1,-0.5,0,0.5,1,2]

# Duration of the force perturbation
PERT_DURATION = 1.5

T_LEV_GRACE = 0.2
MAX_HT_TRIALS = 100
REQ_SUCCESS_RATE = 0.5

# Lever position constants for the small motor (1524 SR) - these assume 11.4
# clicks per degree (4096 CPR encoder in 1x quadrature mode)
threshold_pos = 68 # Used to be 3 degrees
pos1 = 68 # 6 degrees
pos2 = 274 # 24 degrees
drive_name = "118D-D10B" # Name of the external hard drive for video storage

#Mouse variables
mouse_name = ''
mouse_day = datetime.now().day
dt_last_detected = datetime.now()
tt_trial_start = time()
tt_mouse_gone = time()
daily_water = 0
reward_pulls = 0
failed_pulls = 0
mean_ht = 0
median_ht = 0
q3ht = 0
tot_reward_pulls = 0
tot_failed_pulls = 0
tot_days = 0
entrance_rewards = 0
pert_trials = 0
hold_time = 0.0
free_water_remained = 0
ht_trials = 0
phase = 1 # 1 = Training, 2 = Acquisition
pert_force = 0.0

#Booleans

# Indicates if data has been loaded for an animal
data_loaded = False

# Indicates whether a trial is allowed to be started
# (i.e. head is in position, motor is at low torque)
trial_started = False

# Indicates that there is currently a trial ongoing
# (e.g. the lever is in the reward range, or was at last check)
trial_ongoing = False

# Indicates that RFID needs to be double checked
# (either logic signal has gone low or data waiting in buffer)
rfid_interrupt = False

# Indicates that infrared breakbeam has been restored
infrared_restored = False

# Indicates that the mouse is currently nose poking
# (i.e. head is in position)
nose_poke = False

# GPIO Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(G_INFRARED, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(G_RFID, GPIO.IN)
GPIO.setup(G_SOLENOID, GPIO.OUT)
GPIO.setup(G_MOTOR_ENABLE, GPIO.OUT)
GPIO.setup(G_MOTOR_DIRECTION, GPIO.OUT)
GPIO.output(G_MOTOR_DIRECTION, True) # True = CW motor direction

#Motor Constants and Setup
motor = PWM(0) # PWM0 defaults to GPIO18 which is where motor should be connected
motor.export()
motor.period = 1000000 # 1 million ns = 1000Hz
high_motor = 850000 # 85% duty cycle
low_motor = 150000 # 15% duty cycle

#Buzzer Setup
buzzer = PWM(1) # PWM1 defaults to GPIO19 which is where the buzzer should be connected
buzzer.export()

#Camera Constants and Setup
camera = picamera.PiCamera(resolution=(500, 500), framerate = 90) # Initializing camera (EDITED)
stream = picamera.PiCameraCircularIO(camera, seconds=5, bitrate=10000000) # Initializing stream
stream2 = picamera.PiCameraCircularIO(camera, seconds=5, bitrate=10000000) # Initializing stream
camera.vflip = True
camera.hflip = True

# RFID Constants and Setup
port = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout = 1)
RFID_chars = ['0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F']

# Multiprocessing
manager = mp.Manager()
ns = manager.Namespace()
list_lever_pos = manager.list()
ns.trial_started = False
ns.trial_over = False
ns.pos1 = False
ns.pos2 = False
ns.pert = 0
ns.time_in_range = time()
ns.grace_period = False
ns.entered_range = False
ns.pert_force = 0
ns.start_time = None

# Encoder Constants and Setup
CSX = 0 # Pin CS0
CLK = 10000000 # 10Mhz clock freq.
BYTEMD = 4 # Bytes that will be sent to you.
SAMPLING_RATE = 410 # Rate of lever sampling in Hz. 205 gets ~200 Hz rate in practice
lever = Encoder(CSX,CLK,BYTEMD)


def record_lever_pos(_list_lever_pos, _ns):
    '''
    Takes in an empty list and appends the current encoder position and time to it at a certain rate
    (based on specified sampling rate). Also checks if success range/fail range has been reached.
    '''
    global threshold_pos

    if phase == 1:
        threshold_pos = 34
    else:
        threshold_pos = 66

    while True:
        t_last = time()
        position_reading = lever.read_counter()
        position_time = 0
        _list_lever_pos.append([position_time, position_reading,ns.pert])
        if lever.read_counter() > threshold_pos:
            tt_start = time()
            _ns.trial_started = True
            _ns.start_time = tt_start

            # Simpler to make seperate loops for phase 1 and 2
            if phase == 1:
                while True:
                    t_last = time()
                    position_reading = lever.read_counter()
                    position_time = time() - tt_start
                    _list_lever_pos.append([position_time, position_reading])

                    if not _ns.pos1 and position_reading > pos1:
                        tt_range = time()
                        _ns.pos1 = True
                    if _ns.pos1 and position_reading > pos2:
                        _ns.time_in_range = time() - tt_range
                        _ns.pos2 = True
                        _ns.trial_over = True
                        return
                    elif _ns.pos1 and position_reading <= pos1:
                        _ns.time_in_range = time() - tt_range
                        _ns.trial_over = True
                        return
                    elif position_reading <= threshold_pos:
                        _ns.time_in_range = 0.0
                        _ns.trial_over = True
                        return
                    while(time()-t_last < 1.0/SAMPLING_RATE):
                        sleep(0.0001)

            elif phase == 2 or _ns.pert_force ==0:
                print('NO PERTURB')
                while True:
                    t_last = time()
                    position_reading = lever.read_counter()
                    position_time = time() - tt_start
                    _list_lever_pos.append([position_time, position_reading, _ns.pert])
                    if not _ns.pos1 and position_reading > pos1:
                        tt_range = time()
                        _ns.pos1 = True
                    if _ns.pos1 and position_reading > pos2:
                        _ns.time_in_range = time() - tt_range
                        _ns.pos2 = True
                        _ns.trial_over = True
                        return
                    elif _ns.pos1 and position_reading <= pos1:
                        _ns.time_in_range = time() - tt_range
                        _ns.trial_over = True
                        return
                    elif position_reading <= threshold_pos:
                        _ns.time_in_range = 0.0
                        _ns.trial_over = True
                        return
                    while(time()-t_last < 1.0/SAMPLING_RATE):
                        sleep(0.0001)

            elif phase == 3 and _ns.pert_force !=0:
                while True:
                    t_last = time()
                    position_reading = lever.read_counter()
                    position_time = time() - tt_start

                    _list_lever_pos.append([position_time, position_reading, _ns.pert])
                    if not _ns.pos1 and not _ns.entered_range and position_reading > pos1:
                        _ns.entered_range = True
                        tt_range = time()
                        _ns.pos1 = True
                    if _ns.pos1 and position_reading > pos2:
                        _ns.time_in_range = time() - tt_range
                        if not _ns.grace_period and _ns.time_in_range > PERTURBATION_WAIT:
                            _ns.grace_period = True
                            tt_grace = time()
                        if _ns.grace_period and time() - tt_grace > T_LEV_GRACE:
                            _ns.pos2 = True
                            _ns.trial_over = True
                            _ns.grace_period = False
                            return
                        if not _ns.grace_period and _ns.time_in_range < PERTURBATION_WAIT:
                            _ns.pos2 = True
                            _ns.trial_over = True
                            _ns.grace_period = False
                            return
                    elif _ns.pos1 and position_reading <= pos1:
                        _ns.time_in_range = time() - tt_range
                        if not _ns.grace_period and _ns.time_in_range > PERTURBATION_WAIT:
                            _ns.grace_period = True
                            tt_grace = time()
                        if _ns.grace_period and time() - tt_grace > T_LEV_GRACE:
                            _ns.trial_over = True
                            return
                    elif position_reading <= threshold_pos:
                        _ns.time_in_range = 0.0
                        _ns.trial_over = True
                        return
                    while(time()-t_last < 1.0/SAMPLING_RATE):
                        sleep(0.0001)

        while(time()-t_last < 1.0/SAMPLING_RATE):
            sleep(0.0001)


def write_lever_pos(t_event, lever_positions, event):
    """
    Writes lever position data from a trial to the lever position file for a specific mouse.

    Args:
    t_event (datetime): The time of the event.
    lever_positions (list): List of lever positions recorded by record_lever_pos function.
    event (str): String containing the event code.
    mouse_name (str): Name of the mouse.
    threshold_pos (int): Threshold for the lever position.
    """

    file_path = f'data/{mouse_name}/{mouse_name}_leverPos.txt'
    with open(file_path, 'a', encoding='utf-8') as file:
        file.write(f'{t_event}\t{event}\n')

        # Find the first index past the threshold
        first_index = next((i for i, pos in enumerate(lever_positions)
                            if pos[1] > threshold_pos), 0)
        # Adjust index to capture 100 positions before threshold, if possible
        first_index = max(first_index - 100, 0)

        # Write position data to file
        for _time, x_pos, y_pos in lever_positions[first_index:]:
            file.write(f'{_time}\t{x_pos}\t{y_pos}\n')

    print("Lever position saved to file.")



def check_id(tag):
    """
    Checks to see if animal's RFID is in mice.cfg, and if it matches previously detected mouse
    tag is animal RFID
    """
    with open('mice.cfg', 'r', encoding='utf-8') as file:
        mice_lines = file.readlines()

    for mouse_line in mice_lines:
        mouse_line = mouse_line.replace('\n', '')
        mouse_line = mouse_line.split('\t')
        if mouse_line[0] == tag:
            # If data is loaded already, we're checking to
            # see if the mouse is the same as was previously detected
            if data_loaded:
                if mouse_line[1] != mouse_name:
                    print(f'Mouse has changed! {mouse_line[1]}:{mouse_line[0]} \
                          entered the chamber ({datetime.now()}).')
                    return True # Identifies mouse change
                else:
                    print("Mouse has not changed.")
                    return False
            # If data is not loaded, we are just checking to make sure mouse name is in
            # mice.cfg, don't need to return anything
            print(f'{mouse_line[1]}:{mouse_line[0]} entered the chamber ({datetime.now()}).')
            return

    #If check_id gets through the full mice.cfg file without finding the
    # mouse name, should raise a value error
    print("Mouse ID = " + tag)
    raise ValueError("Mouse name not found in mice.cfg.")


def get_serial():
    """
    Reads the most recently detected 12 character RFID input from the serial port
    and ignores all others in the buffer.
    port is a serial object.
    returns a str containing the RFID.
    """

    #Short timeout to ensure that the full serial is detected by the RFID before moving forward
    sleep(0.05)
    #Get the last detected serial input
    while True:
        line = port.readline()
        if port.inWaiting() < 16: #Read out RFIDs until the last one (if there is more than 1)
            break

    #Raise error if nothing was found in the buffer
    if not line:
        raise ValueError("No RFID detected (logical input is high).")

    #Format and return
    rfid = ''
    for c in line:
        if chr(c) in RFID_chars:
            rfid += chr(c)
    port.flushInput()
    return rfid


def record_data(timestamp, event, ht, dt=0.0, _pert_force=0.0):
    """
    Records trial and entrance data to relevant file with the following event codes:
        00: Mouse entered
        99: Mouse exited
        01: Mouse given entrance reward
        02: Successful trial (training phase)
        03: Successful trial - exited forwards
        04: Successful trial - exited backwards
        51: Cancelled trial - nose removed from port mid-trial (note that this
            does not count as a failed trial for success rate purposesand
            lever position/video are not recorded)
        52: Cancelled trial - did not reach success range (note that this
            does not count as a failed trial for success rate purposes and
            lever position/video are not recorded)
        53: Unsuccessful trial - not held long enough in range, exited forwards
        54: Unsuccessful trial - not held long enough in range, exited backwards
        66: Mouse switched to acquisition stage
        77: Mouse advanced up in hold-time
        88: Mouse switched to perturbation stage
    timestamp is a datetime object
    event is a str containing the event code
    ht is the required hold-time during that event (0 in training)
    dt is the duration of the event - this is 0.0 for all non-trial events
    """
    with open(f'data/{mouse_name}/{mouse_name}_data.txt', 'a',
              encoding='utf-8') as file:
        data = str(timestamp) + '\t' + event + '\t' + str(ht) + \
            '\t' + str(round(dt, 4)) + '\t' + str(_pert_force) + '\n'
        file.write(data)


def update_hold_time(_mouse_name, ht, _phase):
    global median_ht, q3ht, mean_ht, hold_time

    if _phase == 1 or _phase == 2 or _phase == 3:
        ht_list=[]
        f_list=[]
        success_codes = ['02','03', '04']
        fail_codes = ['53','54']

        with open(f'data/{_mouse_name}/{_mouse_name}_data.txt',
                  'r', encoding='utf-8') as file:
            holdtimes = file.readlines()

        for x in range(len(holdtimes)-1, 0, -1):
            holdtime = holdtimes[x]
            holdtime = holdtime.replace('\n', '')
            holdtime = holdtime.split('\t')
            t = mktime(strptime(holdtime[0], "%Y-%m-%d %H:%M:%S.%f"))
            if (time() - t)/86400 < 1 :
                if holdtime[1] in success_codes:
                    ht_list.append(float(holdtime[3]))
                elif holdtime[1] in fail_codes:
                    f_list.append(float(holdtime[3]))
            else:
                break

        if len(ht_list) == 0:
            print("No Successful Trials in the past 24 hours.")
            return ht, _phase, 0, 0, 0

        median_ht = round(np.percentile(ht_list,50),2)
        q3ht = round(np.percentile(ht_list + f_list,75),2)
        mean_ht = round(np.mean(ht_list + f_list),2)

        if _phase ==1:
            return 0,1,median_ht,mean_ht, q3ht

        if len(ht_list)>100:
            if ht == MAX_HT and len(ht_list)/(len(ht_list) + len(f_list)) > REQ_SUCCESS_RATE:
                _phase = 3
                hold_time = MAX_HT
                print("Phase 3: Perturbation started")
            elif q3ht > MAX_HT:
                hold_time = MAX_HT
                print("Maximum Hold time reached.")
            elif ht > q3ht:
                hold_time = ht
                print('75th percentile (' +str(q3ht)+ ') is less \
                      than current hold time. The holdtime is not changed.')
            elif len(ht_list)/(len(ht_list) + len(f_list)) > 0.3:
                if q3ht > ht + 0.3:
                    hold_time = ht + 0.1
                    print('75th percentile (' +str(q3ht)+ ') is more than \
                          0.3s longer than current HT. The holdtime is increased by 0.1s.')
                else:
                    hold_time = q3ht
                    print('Hold time set to '+ str(hold_time))
            else:
                holdtime = ht
            return hold_time, _phase, median_ht, mean_ht, q3ht
        else:
            print("Minimum daily water consumption was less than 1 mL. \
                  The Hold time is not changed.")
            return ht, _phase, median_ht, mean_ht, q3ht


def update_mouse_vars(tag):
    """
    Takes an RFID tag and updates mouse variables for the current mouse.
    Re-initializes daily variables if it's past midnight.
    """
    global mouse_name, daily_water, reward_pulls, mouse_day
    global failed_pulls, tot_reward_pulls, tot_failed_pulls, tot_days
    global entrance_rewards, hold_time, ht_trials, phase,pert_trials
    global free_water_remained, mean_ht, median_ht, q3ht

    with open('mice.cfg', 'r', encoding='utf-8') as file:
        mice_lines = file.readlines()

    for mouse_line in mice_lines:
        mouse_line = mouse_line.replace('\n', '')
        mouse_line = mouse_line.split('\t')

        if mouse_line[0] == tag:
            mouse_name = mouse_line[1]
            mouse_day = int(mouse_line[2])
            daily_water = int(mouse_line[3])
            reward_pulls = int(mouse_line[4])
            failed_pulls = int(mouse_line[5])
            tot_reward_pulls = int(mouse_line[6])
            tot_failed_pulls = int(mouse_line[7])
            tot_days = int(mouse_line[8])
            entrance_rewards = int(mouse_line[9])
            hold_time = float(mouse_line[10])
            ht_trials = int(mouse_line[11])
            pert_trials = int(mouse_line[12])
            phase = int(mouse_line[13])
            free_water_remained = int(mouse_line[14])
            now = datetime.now()
            if now.day != mouse_day:
                print("Rollover time reached. Resetting daily variables...")
                old_ht = hold_time
                if phase == 1 or phase == 2 or phase == 3:
                    hold_time, phase, median_ht, mean_ht, q3ht = update_hold_time(mouse_name,
                                                                                  hold_time,phase)
                    ht_trials = 0
                free_water_remained = max(0 , 100 - daily_water)

                record_daily_report(reward_pulls,
                                    failed_pulls,
                                    mean_ht,
                                    median_ht,
                                    q3ht,
                                    free_water_remained,
                                    phase,
                                    old_ht)

                daily_water = 0
                reward_pulls = 0
                failed_pulls = 0
                tot_days += 1

                mouse_day = now.day
            return


def record_daily_report(_reward_pulls,
                        _failed_pulls,
                        _mean_ht, _median_ht,
                        _q3ht,
                        _free_water_remained,
                        _phase,
                        old_ht):
    """
    Record the daily report.
    """
    date = datetime.today() - timedelta(days=1)
    with open(f'data/{mouse_name}/{mouse_name}_dailyreport.txt',
              'a', encoding='utf-8') as file:
        data = (f'{date.date()}\t{_reward_pulls}\t{_failed_pulls}\t{_mean_ht}\t'
                f'{_median_ht}\t{_q3ht}\t{old_ht}\t{_free_water_remained}\t{_phase}\n')
        file.write(data)


def save_mouse_vars():
    """
    Saves mouse variables and writes to file when they leave the chamber.
    """
    with open('mice.cfg', 'r', encoding='utf-8') as file:
        data = file.readlines()

    for idx, row in enumerate(data):
        data[idx] = row.replace('\n', '').split('\t')

        if data[idx][1] == mouse_name:
            data[idx][2] = mouse_day
            data[idx][3] = daily_water
            data[idx][4] = reward_pulls
            data[idx][5] = failed_pulls
            data[idx][6] = tot_reward_pulls
            data[idx][7] = tot_failed_pulls
            data[idx][8] = tot_days
            data[idx][9] = entrance_rewards
            data[idx][10] = hold_time
            data[idx][11] = ht_trials
            data[idx][12] = pert_trials
            data[idx][13] = phase
            data[idx][14] = free_water_remained

    with open('mice.cfg', 'w') as file:
        for row in data:
            newLine = '\t'.join(map(str, row))
            file.write(newLine + '\n')


def kill_lever_pos_proc(p):
    """
    Terminates record_lever_pos.
    """
    while True:
        p.terminate()
        if p.is_alive() == False:
            break


def time_out(aht,rht):
    timeout = 5 - 4*(aht/(rht+0.000001))
    if timeout < 1:
        timeout = 0.5
    return timeout


def dispense_water(t):
    """
    Opens the solenoid for t length of time and then closes it.
    """
    GPIO.output(G_SOLENOID, True)
    sleep(t)
    GPIO.output(G_SOLENOID, False)


def motor_ramp():
    """
    Ramps the motor up from low to high strength over 0.45s. Motor must already be enabled.
    """
    for x in range (11, 90):
        motor.duty_cycle = x*10000
        sleep(0.005)
    motor.duty_cycle = high_motor


def play_tone(freq, n=1, duration=0.2, iti=0.1):
    """
    Plays one or more tones of a specified frequency and duration and inter-tone interval.
    freq is a str indicating either a Hi, Med or Low tone
    n is the number of repetitions (defaults to 1)
    duration is length in seconds of the tone (defaults to 0.2)
    iti is the break between tones in seconds (defaults to 0.1)
    """
    freq_dict = {'Hi': 5000, 'Med': 2500, 'Low': 1000}
    period = int(1000000000/freq_dict[freq])
    duty = int(period/2)
    if buzzer.duty_cycle > period:
        buzzer.duty_cycle = duty
        buzzer.period = period
    else:
        buzzer.period = period
        buzzer.duty_cycle = duty
    while n > 0:
        buzzer.enable = True
        sleep(duration)
        buzzer.enable = False
        n-=1
        if n > 0:
            sleep(iti)


def write_video(timestamp, code):
    """
    Writes video from stream either on an external drive or
    the internal storage; timestamp is the datetime of the trial, code is the trial code.
    """
    print('WRITING VIDEO')
    date = (f"{timestamp.month}-{timestamp.day}-{timestamp.year}-"
            f"{timestamp.hour:02d}{timestamp.minute:02d}{timestamp.second:02d}")

    if stream.tell():
        video_file_before = (
            f'data/{mouse_name}/Videos/'
            f'{mouse_name}_{code}_{date}_BEFORE.h264'
        )
        with open(video_file_before, 'wb') as output:
            stream.copy_to(output)

    if stream2.tell():
        video_file_after = (
            f'data/{mouse_name}/Videos/'
            f'{mouse_name}_{code}_{date}_AFTER.h264'
        )
        with open(video_file_after, 'wb') as output2:
            stream2.copy_to(output2)

    stream.clear()
    stream2.clear()


def check_entrance_reward():
    """
    Checks if animal has reached interval to get another entrance reward (if in training 1)
    """

    with open(f'data/{mouse_name}/{mouse_name}_data.txt',
              'r', encoding='utf-8') as file:
        events = file.readlines()

    for x in range(len(events)-1, 0, -1):
        event = events[x]
        event = event.replace('\n', '')
        event = event.split('\t')

        if event[1] == '01':
            last_drop_time = mktime(strptime(event[0], "%Y-%m-%d %H:%M:%S.%f"))
            interval = time() - last_drop_time
            if interval > T_ENTRANCE_REWARD_INTERVAL:
                print(f"{int(interval)} seconds since last entrance reward given. \
                      Delivering entrance reward.")
                return True
            print(f"Insufficient time has passed since last entrance reward \
                  ({int(interval)} seconds)")
            return False

    print("Delivering first entrance reward.")
    return True

def end_trial(datetime_start, trial_time, _event, outcome, pert_force=0.0):
    """
    Ends a trial when success or fail condition has been met. Returns lever to start position,
    records data and updates variables.
    - datetimeStart is a datetime object
    - trialTime is a float indicating the length of the trial
    - event is a string indicating the event code
    - outcome is a string indicating if trial was successful, failed or cancelled
    """
    global tt_trial_start, tt_mouse_gone
    global trial_started, trial_ongoing, daily_water
    global reward_pulls, failed_pulls, tot_reward_pulls, tot_failed_pulls, ns
    global rfid_interrupt, dt_last_detected, hold_time, ht_trials, MAX_HT_TRIALS, pert_trials, phase

    tt_finished = time()
    motor_ramp() # Returning motor to high torque setting
    record_data(datetime_start, _event, hold_time, trial_time, pert_force) # Recording event
    ns.pos1 = False
    ns.pos2 = False
    ns.grace_period = False
    ns.entered_range = False
    if outcome == 'cancelled':
        camera.stop_preview()
        camera.stop_recording()
        stream2.clear()
        camera.start_recording(stream2, format='h264')
        ns.trial_started = False
        ns.trial_over = False
        ns.pos1 = False
        ns.pos2 = False
        ns.entered_range = False
        ns.grace_period = False
        trial_started = False
        trial_ongoing = False

    # Following section should only be executed if trial was not cancelled
    elif not outcome == 'cancelled':
        # Playing tones, delivering water and updating mouse variables
        if outcome == 'success':
            play_tone('Hi')
            reward_pulls += 1
            tot_reward_pulls += 1
            daily_water += 1
            print(f"Successful trial: code {_event}. {mouse_name} has {reward_pulls} successful \
                   trial(s) today, {tot_reward_pulls} in total ({datetime_start}).")
            dispense_water(T_SOLENOID)
        elif outcome == 'failed':
            play_tone('Low',duration=0.5)
            failed_pulls += 1
            tot_failed_pulls += 1
            print(f"Failed trial: code {_event}. {mouse_name} has {failed_pulls} \
                  failed trial(s) today, {tot_failed_pulls} in total ({datetime_start}).")

        # Stopping video
        camera.stop_preview()
        camera.stop_recording()

        # Recording video and lever position
        write_video(datetime_start, _event)
        write_lever_pos(datetime_start, list_lever_pos, _event)
        stream2.clear()
        camera.start_recording(stream2, format='h264')

        # Checking if phase has changed
        if phase == 1:
            if tot_reward_pulls >= N_TRAINING:
                print(f"{mouse_name} has reached {tot_reward_pulls} \
                      successful trials. Changing to main stage.")
                hold_time, *_ = update_hold_time(mouse_name, INITIAL_HT, phase)
                phase = 2
                record_data(datetime.now(), '66', hold_time)  # 66 is switching to main stage code
        elif phase == 2:
            ht_trials += 1
            # if hold_time < MAX_HT and ht_trials >= N_SUCCESS_CHECK:
            print(f"{mouse_name} has {ht_trials} trials at {hold_time:.2f} second hold time.")
        elif phase == 3:
            ht_trials += 1
            if pert_force != 0:
                pert_trials += 1
                print(f"{mouse_name} has {pert_trials} perturbation trials.")

    # Updating booleans and timing variables, and recording data
    tt_trial_start = time()
    stream.clear()
    ns.trial_started = False
    ns.trial_over = False
    ns.pos1 = False
    ns.pos2 = False
    trial_started = False
    trial_ongoing = False
    ns.entered_range = False
    ns.grace_period = False

    print(f"Recorded trial data and reset variables in {time() - tt_finished} seconds.")

    # During end_trial, it's possible that the mouse may have changed - the
    # rfid_interrupt boolean will trigger a re-check of the RFID if there is
    # data in the serial buffer
    if not rfid_interrupt and port.inWaiting() >= 16:
        print("RFID went out of range during trial.")
        rfid_interrupt = True
        tt_mouse_gone = time()
        dt_last_detected = datetime.now()


def perturbation(pert_force):
    global ns, trial_started, event
    gc.collect ()
    perturbation_duty = int(low_motor + (abs(pert_force))*low_motor)
    trial_started = False
    perturbation_started = False
    perturbation_finished = False
    event = None

    while True: # Checking IR signal for perturbation
        if not ns.trial_over:
            print(f"Perturbation trial initiated. The force is {pert_force * 100:.2f}% of maximum, \
                  latency to perturbation is {PERTURBATION_WAIT} s.")
            tt_perturbation = ns.start_time
            trial_started = True
            while not ns.trial_over:
                position_reading = lever.read_counter()
                if (position_reading < pos1 or position_reading > pos2) and \
                    not perturbation_started:
                    trial_started = False
                elif (not perturbation_started and
                    not perturbation_finished and
                    time() - tt_perturbation >= PERTURBATION_WAIT):
                    if pert_force<0:
                        GPIO.output(G_MOTOR_DIRECTION, False)
                        motor.duty_cycle = perturbation_duty
                        ns.pert = pert_force
                    elif pert_force == 0:
                        ns.pert = 0
                    elif pert_force > 0:
                        motor.duty_cycle = perturbation_duty
                        ns.pert = pert_force
                    perturbation_started = True
                    tt_perturbation = time()
                    print("Perturbation started.")
                elif perturbation_started and time() - tt_perturbation >= PERT_DURATION:
                    motor.duty_cycle = low_motor
                    ns.pert = 0
                    perturbation_started = False
                    perturbation_finished = True
                    GPIO.output(G_MOTOR_DIRECTION, True)
                    print("Perturbation finished.")
                sleep(0.001)

            if perturbation_started:
                #motor.duty_cycle = low_motor
                GPIO.output(G_MOTOR_DIRECTION, True)
                print("Lever went out of range during perturbation. (Code 55)")
                perturbation_started = False
                ns.pert = 0
                event = 55
            elif not perturbation_finished:
                print("Lever went out of range before perturbation started. (Code 56)")
                #motor.duty_cycle = low_motor
                GPIO.output(G_MOTOR_DIRECTION, True)
                perturbation_started = False
                ns.pert = 0
                event = 56
            GPIO.output(G_MOTOR_DIRECTION, True)
            return
        if ns.trial_over:
            print('Perturbation Cancelled!')
            return
        sleep(0.001)


def main():
    global daily_water, entrance_rewards, tt_trial_start, tt_mouse_gone
    global data_loaded, trial_started, trial_ongoing
    global list_lever_pos, ns, mouse_change, rfid_interrupt
    global nose_poke, infrared_restored, dt_last_detected

    global T_TIMEOUT, T_TIMEOUT_CANCELLED, T_TIMEOUT_FAILED
    global T_TIMEOUT_SUCCESS, PERT_DURATION, PERTURBATION_WAIT
    global PERT_LIST, pert_trials, pert_force, free_water_remained

    try:
        # Following loop checks if a mouse is already in the chamber when the program first
        # starts - will activate lever at high torque and sleep until they leave
        if GPIO.input(G_INFRARED) == 0 or GPIO.input(G_RFID) == 1:
            motor.duty_cycle = high_motor
            motor.enable = True
            GPIO.output(G_MOTOR_ENABLE, True)
            print("Mouse in chamber during program start... waiting for it to leave.")
            tt_chamber_clear = time()
            while GPIO.input(G_INFRARED) == 0 or GPIO.input(G_RFID) == 1:
                sleep(0.1)
            motor.duty_cycle = low_motor
            sleep(0.1)
            GPIO.output(G_MOTOR_ENABLE, False)
            motor.enable = False
            print(f"Chamber cleared in {time() - tt_chamber_clear:.2f} seconds.")

        # Need to make sure the lever is in starting position before the encoder count clears
        motor.duty_cycle = low_motor
        motor.enable = True
        GPIO.output(G_MOTOR_ENABLE, True)
        sleep(0.5)
        lever.clear_counter()
        GPIO.output(G_MOTOR_ENABLE, False)
        motor.enable = False

        # Declare some local varibales
        p = None
        t_trial_start = None

        print("Done Init. Waiting for mouse.")
        while True:
            # The following two conditionals are checked at the beginning of every main body
            # loop to determine whether a mouse is in the chamber and what it's identity is
            if GPIO.input(G_RFID) == 1:
                # Following is executed if an RFID has come into range for the first time since
                # the grace period last expired
                if not data_loaded:
                    # Reads off the last detected RFID in the serial
                    # buffer - raises an error if none detected
                    tag_id = get_serial()
                    #Raises an error if mouse is not in mice.cfg
                    check_id(tag_id)
                    motor.duty_cycle = high_motor
                    motor.enable = True
                    GPIO.output(G_MOTOR_ENABLE, True)
                    update_mouse_vars(tag_id) # 00 is the mouse entrance code
                    record_data(datetime.now(), '00', hold_time)
                    data_loaded = True
                    camera.start_recording(stream, format='h264')
                    camera.start_preview()
                    print("Data loaded and motor enabled. Waiting for nose poke...")
                # Following executes if RFID has come back into range within grace
                # period - need to check if it's the same mouse
                elif rfid_interrupt:
                    rfid_interrupt = False
                    print("RFID signal restored. Double checking animal identification.")
                    tag_id = get_serial()

                    # check_id Returns True if last ID in the buffer matches a
                    # different mouse from the currently stored mouse_name
                    if check_id(tag_id):
                        # Save data for previous mouse:
                        camera.stop_recording()
                        stream.clear()
                        record_data(dt_last_detected, '99', hold_time) # 99 is the mouse exit code
                        T_TIMEOUT = T_TIMEOUT_CANCELLED
                        save_mouse_vars()
                        # Reset booleans and load data for new mouse:
                        nose_poke = False
                        update_mouse_vars(tag_id)
                        record_data(datetime.now(), '00', hold_time) # 00 is the mouse entrance code
                        print("Waiting for nose poke...")
                        camera.start_recording(stream, format='h264')
                        camera.start_preview()

            elif GPIO.input(G_RFID) is False:
                # Following evaluates True when a previously detected mouse goes out of range:
                if data_loaded and not rfid_interrupt:
                    print(f"RFID signal for {mouse_name} went out of range.")
                    rfid_interrupt = True
                    # Begins countdown to end of RFID detection grace period
                    tt_mouse_gone = time()
                    # Time that mouse went out of range (used for exit time recording)
                    dt_last_detected = datetime.now()
                # Following evaluates once grace period for mouse detection is over and
                # mouse is assumed to have left the chamber:
                elif rfid_interrupt and time() - tt_mouse_gone > T_RFID_GRACE and not nose_poke:
                    print(f"{mouse_name} not detected for more than {T_RFID_GRACE} seconds. \
                          Assuming chamber is empty ({datetime.now()}).")
                    # Recording exit, saving variables and clearing encoder count:
                    record_data(dt_last_detected, '99', hold_time) # 99 is the mouse exit code
                    T_TIMEOUT = T_TIMEOUT_CANCELLED
                    save_mouse_vars()
                    camera.stop_recording()
                    camera.stop_preview()
                    stream.clear()
                    stream2.clear()
                    tt_lever_back_in_pos = time()
                    while lever.read_counter() > 6: # 6 is equivalent to ~1/2 degree
                        if (time() - tt_lever_back_in_pos) > 2.0:
                            print(f"Possible Error, lever counter value prior to \
                                  clearing is {lever.read_counter()}.")
                            break
                    lever.clear_counter()
                    #Turn off motor and reset booleans
                    motor.duty_cycle = low_motor
                    sleep(0.1)
                    GPIO.output(G_MOTOR_ENABLE, False)
                    motor.enable = False
                    data_loaded = False
                    rfid_interrupt = False
                    nose_poke = False
                else:
                    # This should be the only statement executed any time
                    # there is no mouse in the chamber and the grace period has expired
                    sleep(0.01)

            if not nose_poke:
                if GPIO.input(G_INFRARED) == 0: # Infrared is broken (e.g. a nose poke has occurred)
                    if not data_loaded: # Nose poke detected before RFID was identified
                        print("Infrared broken before RFID was detected. \
                              Waiting until ID can be determined.")
                        tt_rfid_detect = time()
                        while GPIO.input(G_RFID) is False:
                            if time() - tt_rfid_detect > 10:
                                raise ValueError("Infrared broken but RFID could \
                                                 not be detected within 10 seconds.")
                            sleep(0.01)
                        continue #Will loop back to beginning of loop and detect animal
                    print("Infrared broken. Waiting to begin trial.")
                    nose_poke = True
                    tt_ir_broken = time()
                    #Delivers a free water drop for the the nose poke if required
                    if phase == 1 or free_water_remained != 0:
                        if check_entrance_reward():
                            dispense_water(T_SOLENOID)
                            # We want to create association between high tone and
                            # reward delivery, even for free water
                            play_tone('Hi')
                            daily_water += 1
                            entrance_rewards += 1
                            camera.stop_recording()
                            #stream.clear()
                            if phase != 1:
                                free_water_remained -= 1
                            # 01 is nose poke reward code
                            record_data(datetime.now(), '01', hold_time)
                            # If mouse given nose poke drop, wait 5 seconds
                            # until the first trial (otherwise can start immediately)
                            tt_trial_start = time()
                            write_video(datetime.now(),'01')
                            camera.start_recording(stream2,'h264')
            while nose_poke:
                # Animal has removed head from position
                if GPIO.input(G_INFRARED) == 1:
                    if not infrared_restored:
                        tt_trial_cancel = time()
                        infrared_restored = True
                    elif time() - tt_trial_cancel > T_IR_GRACE:
                        if trial_started and not trial_ongoing:
                            kill_lever_pos_proc(p)
                            camera.stop_preview()
                            camera.stop_recording()
                            stream2.clear()
                            write_video(datetime.now(),'50')
                            camera.start_recording(stream2,'h264')
                            motor.duty_cycle = high_motor
                            trial_started = False
                            print("Head removed from position before trial was initiated.")
                        elif trial_ongoing:
                            kill_lever_pos_proc(p)
                            print("Head removed before trial could be completed. \
                                  Trial cancelled (Code 51).")
                            end_trial(
                                t_trial_start,
                                0.0,
                                '51',
                                'cancelled'
                            ) # 51 is trial code for head being removed mid-trial
                            T_TIMEOUT = T_TIMEOUT_CANCELLED
                        else:
                            print("Head removed from position .")
                        nose_poke = False
                        infrared_restored = False
                        break
                elif GPIO.input(G_INFRARED) == 0 and infrared_restored:
                    infrared_restored = False
                    tt_ir_broken = time()

                # Enough time has past since last trial and animal can start the trial
                if not trial_started:
                    if (time() - tt_trial_start) > T_TIMEOUT and \
                        (time() - tt_ir_broken) > T_IR_WAIT:
                        trial_started = True
                        pert_force = random.choice(PERT_LIST)
                        ns.pert_force = pert_force
                        print("Timeout is up, motor switched to low torque.")
                        camera.start_preview()
                        stream2.clear()
                        camera.split_recording(stream2)
                        list_lever_pos = manager.list()
                        p = mp.Process(target=record_lever_pos, args=(list_lever_pos, ns))
                        p.start()
                        motor.duty_cycle = low_motor
                        play_tone('Med')

                # Animal has pulled the lever past threshold
                elif ns.trial_started and not trial_ongoing:
                    t_trial_start = datetime.now()
                    trial_ongoing = True
                    if phase == 1:
                        print("Trial initiated - mouse is in training stage.")
                        pert_force = 0.0
                        ns.pert_force = 0
                    elif phase == 2:
                        print("Trial initiated - mouse is in main stage of testing.")
                        pert_force = 0.0
                        ns.pert_force = 0
                    elif phase == 3:
                        print("Trial initiated - mouse is in perturbation stage.")
                        perturbation(pert_force)

                elif trial_ongoing and ns.trial_over:
                    kill_lever_pos_proc(p)
                    T_TIMEOUT = time_out(ns.time_in_range,hold_time)
                    if phase == 1:
                        # 02 is success code for Training
                        end_trial(t_trial_start, ns.time_in_range, '02', 'success',pert_force=0.0)
                    elif phase == 2 or phase == 3:
                        if ns.pos1:
                            if ns.pos2: # Lever exited range towards back of range
                                if ns.time_in_range >= hold_time:
                                    # 04 is success code for Acquisition
                                    # (exited towards back of range)
                                    end_trial(
                                        t_trial_start,
                                        ns.time_in_range,
                                        '04',
                                        'success',
                                        pert_force
                                    )
                                else:
                                    # 54 is fail code for Acquisition
                                    # (exited towards back of range)
                                    end_trial(
                                        t_trial_start,
                                        ns.time_in_range,
                                        '54',
                                        'failed',
                                        pert_force
                                    )
                            else: # Lever exited range towards front of range
                                if ns.time_in_range >= hold_time:
                                    # 03 is success code for Acquisition
                                    # (exited towards front of range)
                                    end_trial(
                                        t_trial_start,
                                        ns.time_in_range,
                                        '03',
                                        'success',
                                        pert_force
                                    )
                                else:
                                    # 53 is fail code for Acquisition
                                    # (exited towards front of range)
                                    end_trial(
                                        t_trial_start,
                                        ns.time_in_range,
                                        '53',
                                        'failed',
                                        pert_force
                                    )
                        # Lever did not reach goal range - in this case
                        # trial is cancelled and recorded in mouse file, but
                        # lever position not saved etc.
                        else:
                            print("Lever did not reach goal range. Trial cancelled (Code 52).")
                            # 52 is code for trial did not reach goal range (trial cancelled)
                            end_trial(t_trial_start, 0.0, '52', 'cancelled')
                            T_TIMEOUT = T_TIMEOUT_CANCELLED
                    else:
                        T_TIMEOUT = T_TIMEOUT_CANCELLED
                        raise ValueError("Testing phase not found.")
                    ns.pos1 = False
                    ns.pos2 = False
                    ns.entered_range = False
                    ns.grace_period = False
                    break

            # Mouse pulls the lever past threshold at any time outside of a trial
            if not trial_started and lever.read_counter() > threshold_pos:
                if not data_loaded:
                    play_tone('Low',duration=0.5)
                    raise ValueError("Lever moved past threshold before any RFID detected.")
                else:
                    print("Lever moved past threshold during timeout or while head out \
                          of position.")
                    tt_back_in_start_pos = time()
                    while lever.read_counter() > threshold_pos:
                        sleep(0.01)
                    print(f"Lever returned to threshold range in \
                          {time() - tt_back_in_start_pos:.2f} seconds. \
                          Restarting time until trial can begin.")
                    tt_trial_start = time()

    except KeyboardInterrupt:
        print('Closing program...')

    except ValueError as err:
        print(f"Value Error: {err}")
        print("Exit time: "  + str(datetime.now()))
        exit(1)  #Will exit with an error message

    finally:
        if data_loaded:
            if trial_started:
                kill_lever_pos_proc(p)
            record_data(datetime.now(), '99', hold_time) #99 is the mouse exit code
            save_mouse_vars()
            print("Saving mouse data before closing.")
            T_TIMEOUT = T_TIMEOUT_CANCELLED
        motor.duty_cycle = low_motor
        sleep(0.1)
        motor.enable = False
        GPIO.cleanup()
        buzzer.unexport()
        motor.unexport()
        stream.clear()
        camera.close()
        lever.close()


main()
