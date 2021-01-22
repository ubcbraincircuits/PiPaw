"""Program for assessing mice in the piPaw cage with the amplitude methodology.

The main criteria for reward in this version is the amplitude of the pull 
(i.e. the maximum position of the lever during the trial). This testing 
methodology consists of two training phases followed by a testing phase 
and an optional reversal phase. In these different phases, the specific 
criteria required to obtain a water drop reward changes:
    Training 1:     Lever must by pulled past the threshold (3 degrees) and 
                    returned to starting position within 2 seconds. During this 
                    phase, the mouse can also receive one free water drop every 
                    15 minutes by simply nose poking at the port. Mouse is 
                    advanced to Training 2 after a set number of trials.
    Training 2:     Pull amplitude must be >8 degrees and lever must be 
                    returned to starting position within 2 seconds. Mouse is 
                    advanced to the Testing phase after a set number of trials.
    Testing:        Pull amplitude must be within a 12 degree range (15-27 
                    degrees from starting position), and lever must be returned
                    to starting position within 2 seconds.
    Reversal:       If selected as an option (by changing the reversal boolean 
                    value to True), the program will automatically advance mice
                    to a behavioural flexibility assessment based on their
                    performance in the Testing phase. After a set
                    number of trials in Testing, the program will begin to 
                    calculate the mouse's reward rate following every trial 
                    (over a block of previous trials). If this reward rate 
                    passes a certain threshold, they are advanced to the 
                    Reversal phase. Pull amplitude must be within a 7 degree
                    range completely seperate from the testing phase range 
                    (8-15 degrees from starting position), and lever must be 
                    returned to starting position within 2 seconds (as always). 
                    The mouse will stay in this phase until testing is ended by
                    the experimenter.
For all phases, the mouse initiates a trial by nose poking and must leave their
nose in the port during initiation of the trial. Mice proceed at their own pace
through the task in a group housed environment. Trial timing, outcome, lever
position data and video is collected for all trials.

Written by Cameron Woodard
"""

import serial
import picamera
import classReadEncoder
import multiprocessing as mp
import RPi.GPIO as GPIO
from time import time, sleep, mktime, strptime
from datetime import datetime
from pwm import PWM
from sys import exit

# Task Constants
TRAINING_1_LENGTH = 100
TRAINING_2_LENGTH = 100
SOLENOID_OPEN_TIME = 0.2
REVERSE_SOLENOID = False
TRIAL_TIMEOUT = 5.0
TRIAL_TIME_LIMIT = 2.0
IR_WAIT_TIME = 1.0
RFID_GRACE_PERIOD = 30.0
IR_GRACE_PERIOD = 0.2
ENTRANCE_REWARD_INTERVAL = 900

# Reversal Constants
ASSESS_REVERSAL = True
MIN_DAYS_BEFORE_REVERSAL = 7
MIN_TRIALS_BEFORE_REVERSAL = 1000 
MAX_TRIALS_BEFORE_REVERSAL = 2200 
CHECK_SUCCESS_BIN = 200
SUCCESS_RATIO_REQUIREMENT = 0.7


# Lever Position Constants
# These values use the 30 degree range (small motor) and assume 11.4 clicks per
# degree (4096 clicks per revolution)
THRESHOLD_POS = 34 # 3 degrees
POS_1 = 91 # 8 degrees
POS_2 = 171 # 15 degrees
POS_3 = 308 # 27 degrees

# File Path Constants
DATA_PATH = '/home/pi/piPaw/'

# GPIO Pin Constants
G_SOLENOID = 4
G_RFID = 16
G_INFRARED = 21
G_MOTOR_ENABLE = 23
G_MOTOR_DIRECTION = 24

# Motor Constants
PWM_PERIOD = 1000000  # 1 million ns = 1000Hz
HI_DUTY_CYCLE = 900000  # 90% duty cycle
LOW_DUTY_CYCLE = 100000  # 10% duty cycle

# Encoder Constants
ENCODER_CSX = 0 # Pin CS0
ENCODER_CLK = 10000000 # 10Mhz clock frequency
ENCODER_BYTE_MODE = 4 # Bytes that will be sent to you.
ENCODER_SAMPLING_RATE = 205 # 205 gets ~200 Hz rate in practice

# Camera Constants
CAMERA_RESOLUTION = (256, 256)
CAMERA_FRAMERATE = 30
STREAM_SECONDS = 3
STREAM_BITRATE = 700000
CAMERA_VERTICAL_FLIP = True
CAMERA_HORIZONTAL_FLIP = True

# RFID Constants
RFID_CHARS = ['0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F']
RFID_PORT = '/dev/ttyUSB0'
RFID_BAUDRATE = 9600
RFID_TIMEOUT = 1

# Buzzer Constants
HI_TONE = 2400
MED_TONE = 1200
LOW_TONE = 400

# Global Mouse Variables
mouse_name = ''
mouse_day = 0
daily_water = 0
rewarded_trials = 0
failed_trials = 0
total_rewarded_trials = 0
total_failed_trials = 0
days_of_testing = 0
nose_poke_water = 0
testing_phase = 0  # 1 = Training 1, 2 = Training 2, 3 = Testing, 4 = Reversal


def record_lever_positions(lever, lever_positions, ns):
    """Records encoder position and time to an empty list and checks if 
    reward/fail ranges have been reached.
    
    Args:
        lever is a classReadEncoder.Encoder() object.
        lever_positions is a multiprocessing.Manager().list() object.
        ns is a multiprocessing.Manager().Namespace() object.
        
    Returns:
        This function is spawned as a seperate process by multiprocessing and
        does not return anything to main(). The function will run indefinitely
        until it is terminated by multiprocessing.
    """
    while True:
        if lever.readCounter() > THRESHOLD_POS:
            tt_start = time()
            ns.trial_started = True
            while True:
                t_last = time()
                position_reading = lever.readCounter()
                if position_reading <= THRESHOLD_POS:
                    ns.trial_ended = True
                    return
                datBuffer = (str(time()-tt_start) + '\t' + 
                             str(position_reading) + '\n')
                lever_positions.append(datBuffer)
                if not ns.pos1 == True and position_reading > POS_1:
                    ns.pos1 = True
                if not ns.pos2 == True and position_reading > POS_2:
                    ns.pos2 = True
                if not ns.pos3 == True and position_reading > POS_3:
                    ns.pos3 = True
                while(time()-t_last < 1.0/ENCODER_SAMPLING_RATE):
                    sleep(0.0001)
        sleep(0.0001)

def write_lever_positions(timestamp, lever_positions, trial_code):
    """Appends the lever position data from one trial into the leverPos text 
    file for that animal.
    
    Args:
        timestamp is a datetime.datetime() object indicating time of trial
        lever_positions is a list of strings of timestamped lever positions 
            created by record_lever_positions()
        trial_code is a string containing the event code
        
    Returns:
        None.
    """   
    with open(DATA_PATH + '/data/%s/%s_leverPos.txt' 
              %(mouse_name, mouse_name), 'a') as file:
        initial_line = str(timestamp) + '\t' + trial_code + '\n'
        file.write(initial_line)
        for lever_position in lever_positions:
            file.write(lever_position)
    print("Lever position saved to file.")

def get_rfid(port):
    """Reads the most recently detected 12 character RFID input from the 
    serial port and ignores all others in the buffer.
    
    Args:
        port is a serial.Serial() object.
    
    Returns:
        A string containing the RFID.
        
    Raises:
        ValueError: No RFID could be read despite TIR pin going High.
    """
    sleep(0.05)  # Short timeout to ensure that the full RFID is detected
    while True:  # Read out RFIDs until the last one (if there is more than 1) 
        line = port.readline()
        if port.inWaiting() < 16: 
            break
    # Raise error if nothing was found in the buffer
    if not line:
        raise ValueError("RFID could not be read.")
    # Format and return
    rfid = ''
    for c in line:
        if chr(c) in RFID_CHARS:
            rfid += chr(c)
    port.flushInput()
    return rfid

def update_mouse_variables(rfid_tag):
    """Takes an RFID tag and updates mouse variables for the current mouse.
    
    Args:
        rfid_tag is a string containing the mouse RFID.
    
    Returns:
        None.
        
    Raises:
        ValueError: rfid_tag not found in the mice.cfg file.
    
    """
    global mouse_name, mouse_day, daily_water, rewarded_trials
    global failed_trials, total_rewarded_trials, total_failed_trials
    global days_of_testing, nose_poke_water, testing_phase
    
    with open(DATA_PATH + 'mice.cfg', 'r') as file:
        mice_lines = file.readlines()
    
    for mouse_line in mice_lines:
        mouse_line = mouse_line.replace('\n', '')
        mouse_line = mouse_line.split('\t')
        if mouse_line[0] == rfid_tag:
            print("%s:%s entered the chamber (%s)." %(mouse_line[1], 
                  mouse_line[0], str(datetime.now())))
            mouse_name = mouse_line[1]
            mouse_day = int(mouse_line[2])
            daily_water = int(mouse_line[3])
            rewarded_trials = int(mouse_line[4])
            failed_trials = int(mouse_line[5])
            total_rewarded_trials = int(mouse_line[6])
            total_failed_trials = int(mouse_line[7])
            days_of_testing = int(mouse_line[8])
            nose_poke_water = int(mouse_line[9])
            testing_phase = int(mouse_line[10])
            if datetime.now().day != mouse_day:
                print("Rollover time reached. Resetting daily variables...")
                daily_water = 0
                rewarded_trials = 0
                failed_trials = 0
                days_of_testing += 1
                mouse_day = datetime.now().day
            return
    
    print("Mouse ID = " + rfid_tag)
    raise ValueError("RFID not found in mice.cfg.")

def check_mouse_id(rfid_tag):
    """Checks if detected RFID matches the previously detected mouse.
    
    Args:
        rfid_tag is a string containing the mouse RFID.
    
    Returns:
        True if a new mouse was detected in the grace period, or False if the 
        mouse has not changed. 
        
    Raises:
        ValueError: rfid_tag not found in the mice.cfg file.
    """
    with open(DATA_PATH + 'mice.cfg', 'r') as file:
        mice_lines = file.readlines()        
    
    for mouse_line in mice_lines:
        mouse_line = mouse_line.replace('\n', '')
        mouse_line = mouse_line.split('\t')
        if mouse_line[0] == rfid_tag:
            if mouse_line[1] != mouse_name:
                print("Mouse has changed. Determining ID...")
                return True
            else:
                print("Mouse has not changed.")
                return False       
    
    print("Mouse ID = " + rfid_tag)
    raise ValueError("RFID not found in mice.cfg.")
        
def record_event(timestamp, event, trial_duration=0.0):
    """Records trial and entrance events to the event file for that animal.
    
    The following event codes are used:
        00: Mouse entered
        99: Mouse exited
        01: Mouse given nose-poke reward
        02: Rewarded trial (training phase 1)
        03: Rewarded trial (training phase 2)
        04: Rewarded trial (testing phase)
        05: Rewarded trial (reversal phase)
        10: Failed trial - held too long (training 1 phase)
        11: Failed trial - undershot (training 2 phase)
        12: Failed trial - held too long (training 2 phase)
        13: Failed trial - undershot (testing phase)
        14: Failed trial - overshot (testing phase)
        15: Failed trial - held too long (testing phase)
        16: Failed trial - undershot (reversal phase)
        17: Failed trial - overshot (reversal phase)
        18: Failed trial - held too long (reversal phase)
        66: Mouse switched to second training phase
        77: Mouse switched to initial testing phase
        88: Mouse switched to reversal phase
    
    Args:
        timestamp is a datetime.datetime() object.
        event is a string containing the event code.
        trial_duration is a float indicating the duration of the event.
        
    Returns:
        None.
    """
    with open(DATA_PATH + 'data/%s/%s_data.txt' 
              %(mouse_name, mouse_name), 'a') as file:
        event_line = (str(timestamp) + '\t' + event + '\t' + 
                      str(round(trial_duration, 4)) + '\n')
        file.write(event_line)

def save_mouse_variables():
    """Writes mouse variables to mice.cfg when they leave the chamber.
    
    Args:
        None.
    
    Returns:
        None.
    """
    with open(DATA_PATH + 'mice.cfg', 'r') as file:
        data = file.readlines()

    for j in range(len(data)):
        data[j] = data[j].replace('\n', '')
        data[j] = data[j].split('\t')
        
        if data[j][1] == mouse_name:
            data[j][2] = mouse_day
            data[j][3] = daily_water
            data[j][4] = rewarded_trials
            data[j][5] = failed_trials
            data[j][6] = total_rewarded_trials
            data[j][7] = total_failed_trials
            data[j][8] = days_of_testing
            data[j][9] = nose_poke_water
            data[j][10] = testing_phase
    
    with open(DATA_PATH + 'mice.cfg', 'w') as file:
        for j in range(len(data)):
            newLine = ''
            for i in range(len(data[j])):
                newLine += (str(data[j][i]) 
                    + ('\n' if i == (len(data[j])-1) else '\t'))
            file.write(newLine)
            
def kill_process(process):
    """Terminates a process and ensures it is no longer alive.
    
    Args:
        process is a multiprocessing.Process() object
    
    Returns:
        None.
    """
    while True:
        process.terminate()
        if process.is_alive() == False:
            break

def dispense_water(t):
    """Opens and then closes the solenoid valve.
    
    Args:
        t is a float indicating the length of time to open the valve for.
        
    Returns:
        None
    """
    if not REVERSE_SOLENOID:
        GPIO.output(G_SOLENOID, GPIO.HIGH)
        sleep(t)
        GPIO.output(G_SOLENOID, GPIO.LOW)
    else:
       GPIO.output(G_SOLENOID, GPIO.LOW)
       sleep(t)
       GPIO.output(G_SOLENOID, GPIO.HIGH) 
    
def motor_ramp(motor):
    """Ramps the motor up from low to high strength over 0.45s. Motor must 
    already be enabled.
    
    Args:
        motor is a pwm.PWM object.
        
    Returns:
        None.
    """
    for x in range (11, 90):
        motor.duty_cycle = x*10000
        sleep(0.005)
    motor.duty_cycle = HI_DUTY_CYCLE

def play_tone(buzzer, frequency, n=1, duration=0.2, iti=0.1):
    """Plays one or more tones of a specified frequency, duration and ITI.
    
    Args:
        buzzer is a pwm.PWM() object.
        frequency is a an int indicating the tone frequency in Hz.
        n is the number of tone repetitions.
        duration is a float indicating the length in seconds of the tone.
        iti is a float indicating the inter-tone interval in seconds.
        
    Returns:
        None.
    """
    period = int(1000000000/frequency)
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
        
def write_video(stream, timestamp, code):
    """Writes video from stream. 
    
    Args:
        stream is a picamera.PiCameraCircularIO() video stream.
        timestamp is a datetime.datetime() object indicating trial time.
        code is a string indicating the trial code.
        
    Returns:
        None.
    """
    date = (str(timestamp.month) + '-' + str(timestamp.day) + '-' 
            + str(timestamp.year) + '-' +str (timestamp.hour) 
            + str(timestamp.minute) + str(timestamp.second))    
    with open(DATA_PATH + 'data/%s/Videos/%s_%s_%s.h264' 
              %(mouse_name, mouse_name, code, date), 'wb') as output:
        stream.copy_to(output)
    
def check_nosepoke_reward():
    """Checks if animal has reached fixed interval to obtain another nose-poke
    water reward (only in Training 1 phase).
    
    Args:
        None.
    
    Returns:
        True if the mouse should receive a nose-poke reward, or False if
        insufficient time has passed.
    """
   
    with open(DATA_PATH + 'data/%s/%s_data.txt' 
              %(mouse_name, mouse_name), 'r') as file:
        events = file.readlines()
    
    for x in range(len(events)-1, 0, -1):
        event = events[x]
        event = event.replace('\n', '')
        event = event.split('\t')
        
        if event[1] == '01':
            last_drop_time = mktime(strptime(event[0], "%Y-%m-%d %H:%M:%S.%f"))
            interval = time() - last_drop_time
            if interval > ENTRANCE_REWARD_INTERVAL:
                print("%i seconds since last entrance reward given. "
                      "Delivering entrance reward." %(int(interval)))
                return True
            print("Insufficient time has passed since last entrance reward "
                  "(%i seconds)" %(int(interval)))
            return False
    print("Delivering first entrance reward.")
    return True
    

def check_reversal():
    """Checks if animal has met success rate criteria to advance to reversal.
    
    Args:
        None.
        
    Returns:
        True if criteria is met, False otherwise.
    """
    fail_codes = ['13', '14', '15']
    successes = 0
    total = 0
    
    with open(DATA_PATH + 'data/%s/%s_data.txt' 
              %(mouse_name, mouse_name), 'r') as file:
        events = file.readlines()
        
    for x in range(len(events)-1, 0, -1):
        event = events[x]
        event = event.replace('\n', '')
        event = event.split('\t')
        
        if event[1] == '04':
            successes += 1
            total += 1
        elif event[1] in fail_codes:
            total += 1
        
        if total == CHECK_SUCCESS_BIN:
            break
        
    if successes / total >= SUCCESS_RATIO_REQUIREMENT:
        print("%s has met criteria for switch to reversal phase (%i successes "
              "out of %i total)." %(mouse_name, successes, total))
        record_event(datetime.now(), '88') #88 is switching to reversal code
        return True
    else:
        print("%s has not met criteria for switch to reversal phase (%i "
              "successes out of %i total)." %(mouse_name, successes, total))
        return False

                
def main():
    """Initializes and runs the piPaw system and amplitude testing 
    methodology.
    
    Args:
        None.
    
    Returns:
        None.
    """
    global daily_water, rewarded_trials, failed_trials
    global total_rewarded_trials, total_failed_trials, days_of_testing
    global nose_poke_water, testing_phase
    
    # Initialize Boolean Variables
    data_is_loaded = False
    trial_can_start = False
    rfid_signal_interrupted = False
    infrared_signal_restored = False
    infrared_is_broken = False
    
    try:
        print("Initializing sensors and motor...")
        
        # GPIO Setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(G_INFRARED, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(G_RFID, GPIO.IN)
        GPIO.setup(G_SOLENOID, GPIO.OUT)
        if REVERSE_SOLENOID:
            GPIO.output(G_SOLENOID, GPIO.HIGH)
        GPIO.setup(G_MOTOR_ENABLE, GPIO.OUT)
        GPIO.setup(G_MOTOR_DIRECTION, GPIO.OUT)
        GPIO.output(G_MOTOR_DIRECTION, GPIO.HIGH) # High = CW motor direction
        
        # Motor Setup
        motor = PWM(0)  # PWM0 defaults to GPIO18
        motor.export()
        motor.period = PWM_PERIOD 
        
        # Encoder Setup
        lever = classReadEncoder.Encoder(ENCODER_CSX,ENCODER_CLK,
                                         ENCODER_BYTE_MODE)
        
        # Camera Setup
        camera = picamera.PiCamera(resolution=CAMERA_RESOLUTION, 
                                   framerate=CAMERA_FRAMERATE)
        stream = picamera.PiCameraCircularIO(camera, seconds=STREAM_SECONDS, 
                                             bitrate=STREAM_BITRATE)
        camera.vflip = CAMERA_VERTICAL_FLIP
        camera.hflip = CAMERA_HORIZONTAL_FLIP
        
        # RFID Setup
        port = serial.Serial(RFID_PORT, baudrate=RFID_BAUDRATE, 
                             timeout=RFID_TIMEOUT)
        
        # Buzzer Setup
        buzzer = PWM(1)  # PWM1 defaults to GPIO19
        buzzer.export()
        
        # Multiprocessing Setup
        manager = mp.Manager()
        ns = manager.Namespace()
        lever_positions = manager.list()
        ns.trial_started = False
        ns.trial_ended = False
        ns.pos1 = False
        ns.pos2 = False
        ns.pos3 = False
        
        # If a mouse is already in the chamber, activate lever at high torque
        # and sleep until they leave
        if (GPIO.input(G_INFRARED) == GPIO.LOW 
            or GPIO.input(G_RFID) == GPIO.HIGH):
            motor.duty_cycle = HI_DUTY_CYCLE
            motor.enable = True
            GPIO.output(G_MOTOR_ENABLE, GPIO.HIGH)
            print("Mouse in chamber during program start... "
                  "waiting for it to leave.")
            tt_chamber_empty = time()
            while (GPIO.input(G_INFRARED) == GPIO.LOW 
                   or GPIO.input(G_RFID) == GPIO.HIGH):
                sleep(1)
            motor.duty_cycle = LOW_DUTY_CYCLE
            sleep(0.1)
            GPIO.output(G_MOTOR_ENABLE, GPIO.LOW)
            motor.enable = False
            print("Chamber cleared in %f seconds." %(time()-tt_chamber_empty))
        
        # Ensure lever is in starting position before the encoder count clears
        motor.duty_cycle = LOW_DUTY_CYCLE
        motor.enable = True
        GPIO.output(G_MOTOR_ENABLE, GPIO.HIGH)
        sleep(0.5)
        lever.clearCounter()
        GPIO.output(G_MOTOR_ENABLE, GPIO.LOW)
        motor.enable = False
        
        # Initialize timing variables
        tt_trial_start = time()
        datetime_last_detected = datetime.now()
        
        print("Done piPaw initialization. Waiting for mouse.")
        
        while True:
            
            # Following two conditionals are checked every loop to determine 
            # if a mouse is in the chamber, or if a previously identified
            # mouse has left the chamber.
            
            if GPIO.input(G_RFID) == GPIO.HIGH: #Tag in range of reader
                if not data_is_loaded: 
                    rfid_tag = get_rfid(port)
                    motor.duty_cycle = HI_DUTY_CYCLE
                    motor.enable = True
                    GPIO.output(G_MOTOR_ENABLE, GPIO.HIGH)
                    update_mouse_variables(rfid_tag)
                    record_event(datetime.now(), '00')
                    data_is_loaded = True
                    print("Data loaded and motor enabled. "
                          "Waiting for nose poke...")
                elif rfid_signal_interrupted: 
                    rfid_signal_interrupted = False
                    print("RFID signal restored. Double checking animal "
                          "identification.")
                    rfid_tag = get_rfid(port)
                    if check_mouse_id(rfid_tag):  # True if new mouse detected
                        record_event(datetime_last_detected, '99')
                        save_mouse_variables()
                        infrared_is_broken = False
                        update_mouse_variables(rfid_tag)
                        record_event(datetime.now(), '00')
                        print("Waiting for nose poke...")                    
        
            elif GPIO.input(G_RFID) == GPIO.LOW:  # No tag in range of reader
                if data_is_loaded and not rfid_signal_interrupted: 
                    print("RFID signal for %s went out of range." 
                          %(mouse_name))
                    rfid_signal_interrupted = True
                    tt_mouse_gone = time() 
                    datetime_last_detected = datetime.now()
                elif (rfid_signal_interrupted 
                      and time()-tt_mouse_gone > RFID_GRACE_PERIOD 
                      and not infrared_is_broken):
                    print("%s not detected for more than %i seconds. "
                          "Assuming chamber is empty (%s)." %(mouse_name, 
                          RFID_GRACE_PERIOD, str(datetime.now())))
                    record_event(datetime_last_detected, '99')
                    save_mouse_variables()
                    tt_lever_start_position = time()
                    while lever.readCounter() > 6:  # 6 is equivalent to .5 deg
                        if time()-tt_lever_start_position > 2.0:
                            print("Possible error, lever counter value prior "
                                  "to clearing is %i." %(lever.readCounter()))
                            break
                    lever.clearCounter()
                    motor.duty_cycle = LOW_DUTY_CYCLE
                    sleep(0.1)
                    GPIO.output(G_MOTOR_ENABLE, GPIO.LOW)
                    motor.enable = False
                    data_is_loaded = False
                    rfid_signal_interrupted = False
                    infrared_is_broken = False
                else:
                    # This sleep is the only statement executed any time there 
                    # is no mouse in the chamber and grace period has expired
                    sleep(0.01)
            
            # Following two conditionals check the value of the boolean 
            # infrared_is_broken against the GPIO value of the IR breakbeam in
            # order to determine whether the mouse has nose-poked for enough
            # time to trigger a trial start. If so, the trial start conditions 
            # are triggered and a trial is recorded if the mouse initiates.
            
            if not infrared_is_broken: 
                if GPIO.input(G_INFRARED) == GPIO.LOW: # Infrared is broken
                    if not data_is_loaded:
                        print("Infrared broken before RFID was detected. "
                              "Waiting until ID can be determined.")
                        tt_rfid_detected = time()
                        while GPIO.input(G_RFID) == GPIO.LOW:
                            if time() - tt_rfid_detected > 10:
                                raise ValueError("Infrared broken but RFID "
                                    "could not be detected within 10 seconds.")
                            sleep(0.01)
                        continue
                    print("Infrared broken. Waiting to begin trial.")
                    infrared_is_broken = True
                    tt_infrared_broken = time()
                    if testing_phase == 1:
                        if check_nosepoke_reward():
                            dispense_water(SOLENOID_OPEN_TIME)
                            play_tone(buzzer, HI_TONE, n=2)
                            daily_water += 1
                            nose_poke_water += 1
                            record_event(datetime.now(), '01')
                            tt_trial_start = time()
                
            if infrared_is_broken:
                if GPIO.input(G_INFRARED) == GPIO.HIGH:  # Head out of position
                    if not infrared_signal_restored:
                        tt_trial_cancel = time()
                        infrared_signal_restored = True
                    elif time() - tt_trial_cancel > IR_GRACE_PERIOD:
                        if trial_can_start:
                            print("Head removed from position before trial "
                                  "was initiated.")
                            kill_process(record_lever)
                            camera.stop_preview()
                            camera.stop_recording()
                            stream.clear()
                            motor.duty_cycle = HI_DUTY_CYCLE
                            trial_can_start = False
                        else:
                            print("Head removed from position .")
                        infrared_is_broken = False
                        infrared_signal_restored = False
                        continue
                elif (GPIO.input(G_INFRARED) == GPIO.LOW 
                      and infrared_signal_restored):
                    infrared_signal_restored = False
                
                # Enough time has past since last trial
                if not trial_can_start:
                    if (time() - tt_trial_start > TRIAL_TIMEOUT and 
                            time() - tt_infrared_broken > IR_WAIT_TIME):
                        trial_can_start = True    
                        print("Trial timeout is up, motor switched to low "
                              "torque.")
                        camera.start_preview()
                        camera.start_recording(stream, format='h264')
                        record_lever = mp.Process(
                                target=record_lever_positions, 
                                args=(lever, lever_positions, ns))
                        play_tone(buzzer, MED_TONE, duration=0.1)
                        record_lever.start()
                        motor.duty_cycle = LOW_DUTY_CYCLE
                    
                # Mouse has initiated trial
                if ns.trial_started:
                    dt_trial_start = datetime.now()
                    tt_trial_end = time()
                    reward_reached = False
                    fail_reached = False
                    if testing_phase == 1:
                        print("Trial initiated - mouse is in first training "
                              "phase.")
                        play_tone(buzzer, HI_TONE, duration=0.1)
                    elif testing_phase == 2:
                        print("Trial initiated - mouse is in second training "
                              "phase.")
                    elif testing_phase == 3:
                        print("Trial initiated - mouse is in testing "
                              "phase.")
                    elif testing_phase == 4:
                        print("Trial initiated - mouse is in reversal phase.")
                    # Goes into following loop until trial time limit reached
                    while (time() - tt_trial_end) < TRIAL_TIME_LIMIT:
                        if testing_phase == 2:
                            if not reward_reached and ns.pos1:
                                print("Reward zone reached.")
                                play_tone(buzzer, HI_TONE, duration=0.1)
                                reward_reached = True
                        elif testing_phase == 3:
                            if not reward_reached and ns.pos2:
                                print("Reward zone reached.")
                                play_tone(buzzer, HI_TONE, duration=0.1)
                                reward_reached = True
                            elif not fail_reached and ns.pos3:
                                print("Fail zone reached. Waiting for return "
                                      "to threshold or time limit reached.")
                                play_tone(buzzer, LOW_TONE, duration=0.1)
                                fail_reached = True
                        elif testing_phase == 4:
                            if not reward_reached and ns.pos1:
                                print("Reward zone reached.")
                                play_tone(buzzer, HI_TONE, duration=0.1)
                                reward_reached = True
                            if not fail_reached and ns.pos2:
                                print("Fail zone reached. Waiting for return "
                                      "to threshold or time limit reached.")
                                play_tone(buzzer, LOW_TONE, duration=0.1)
                                fail_reached = True
                        if ns.trial_ended: # Lever below threshold
                            trial_duration = time() - tt_trial_end
                            kill_process(record_lever)
                            if testing_phase == 1:
                                trial_code = '02'
                                is_trial_rewarded = True
                            elif testing_phase == 2:
                                if ns.pos1:
                                    trial_code = '03'
                                    is_trial_rewarded = True
                                else:
                                    trial_code = '11'
                                    is_trial_rewarded = False
                            elif testing_phase == 3:
                                if ns.pos2 and not ns.pos3:
                                    trial_code = '04'
                                    is_trial_rewarded = True
                                elif ns.pos3:
                                    trial_code = '14'
                                    is_trial_rewarded = False
                                else:
                                    trial_code = '13'
                                    is_trial_rewarded = False
                            elif testing_phase == 4:
                                if ns.pos1 and not ns.pos2:
                                   trial_code = '05'
                                   is_trial_rewarded = True
                                elif ns.pos2:
                                    trial_code = '17'
                                    is_trial_rewarded = False
                                else:
                                    trial_code = '16'
                                    is_trial_rewarded = False
                            else:
                                raise ValueError("Testing phase not found.")
                            break    
                    else: # Trial time limit reached
                        trial_duration = TRIAL_TIME_LIMIT
                        kill_process(record_lever)
                        if testing_phase == 1:
                            trial_code = '10'
                            is_trial_rewarded = False
                        elif testing_phase == 2:
                            trial_code = '12'
                            is_trial_rewarded = True
                        elif testing_phase == 3:
                            trial_code = '15'
                            is_trial_rewarded = False
                        elif testing_phase == 4:
                            trial_code = '18'
                            is_trial_rewarded = False
                        else:
                            raise ValueError("Testing phase not found.")
                    
                    # End the trial, record all data, reset variables and
                    # check to see if the phase has changed                    
                    motor_ramp(motor)
                    if is_trial_rewarded:
                        play_tone(buzzer, HI_TONE, n=2)
                        dispense_water(SOLENOID_OPEN_TIME)
                        rewarded_trials += 1
                        total_rewarded_trials += 1
                        daily_water += 1
                        print("Rewarded trial. %s has %i rewarded "
                              "trial(s) today, %i in total (" %(mouse_name, 
                              rewarded_trials, total_rewarded_trials) 
                              + str(dt_trial_start) + ").")
                    else:
                        play_tone(buzzer, LOW_TONE, n=2)
                        failed_trials += 1
                        total_failed_trials += 1
                        print("Failed trial: code %s. %s has %i failed " 
                              "trial(s) today, %i in total (" %(trial_code, 
                              mouse_name, failed_trials, total_failed_trials) 
                              + str(dt_trial_start) + ").")
                    camera.stop_preview()
                    camera.stop_recording()
                    write_video(stream, dt_trial_start, trial_code)
                    stream.clear()
                    tt_trial_start = time()
                    record_event(dt_trial_start, trial_code, trial_duration)
                    write_lever_positions(dt_trial_start, lever_positions, 
                                          trial_code)
                    lever_positions = manager.list()
                    ns.trial_started = False
                    ns.trial_ended = False
                    ns.pos1 = False
                    ns.pos2 = False
                    ns.pos3 = False
                    trial_can_start = False
                    if testing_phase == 1:
                        if total_rewarded_trials >= TRAINING_1_LENGTH:
                            print("%s has reached %i rewarded trials. " 
                                  "Changing to second training phase." 
                                  %(mouse_name, total_rewarded_trials))
                            record_event(datetime.now(), '66')
                            testing_phase = 2
                    elif testing_phase == 2:
                        if (total_rewarded_trials >= (TRAINING_1_LENGTH + 
                            TRAINING_2_LENGTH)):
                            print("%s has reached %i rewarded trials. "
                                  "Changing to testing phase." 
                                  %(mouse_name, total_rewarded_trials))
                            record_event(datetime.now(), '77')
                            testing_phase = 3
                    elif testing_phase == 3 and ASSESS_REVERSAL:
                        if (days_of_testing >= MIN_DAYS_BEFORE_REVERSAL 
                            and total_rewarded_trials >= 
                                MIN_TRIALS_BEFORE_REVERSAL 
                            and total_rewarded_trials < 
                                MAX_TRIALS_BEFORE_REVERSAL):
                            print("%s has %i rewarded trials and min. %i " 
                                  "days in the cage. Checking if reversal " 
                                  "criteria has been met." %(mouse_name, 
                                  total_rewarded_trials, 
                                  MIN_DAYS_BEFORE_REVERSAL))
                            if check_reversal():
                                testing_phase = 4
                    print("Recorded trial data and reset variables.")
                    
                    # While ending the trial and cleaning up it's possible 
                    # that the mouse has changed, so check to see if there is
                    # anything in the serial buffer
                    if not rfid_signal_interrupted and port.inWaiting() >= 16:
                        print("RFID went out of range during trial.")
                        rfid_signal_interrupted = True
                        tt_mouse_gone = time()
                        datetime_last_detected = datetime.now()
                                        
            # If the mouse pulls the lever at any time when a trial is not
            # allowed to start
            if not trial_can_start and lever.readCounter() > THRESHOLD_POS:
                if not data_is_loaded:
                    raise ValueError("Lever moved past threshold before any " 
                                     "RFID detected.")
                else:
                    print("Lever moved past threshold during timeout or while "
                          "head out of position.")
                    tt_lever_start_position = time()
                    while lever.readCounter() > THRESHOLD_POS:
                        sleep(0.01)
                    print("Lever returned to threshold range in %f seconds. "
                          "Restarting time until trial can begin." 
                          %(time()-tt_lever_start_position))
                    tt_trial_start = time()
    
    except KeyboardInterrupt:
        print('Keyboard interrupt detected. Closing program.')  
        
    except ValueError as e:
        print("Value Error: {0}".format(e))
        print("Exit time: "  + str(datetime.now()))
        exit(1)  # Will trigger reboot of program by monitor.sh
    
    finally:
        if data_is_loaded:
            print("Saving mouse data before closing.")
            if trial_can_start:
                kill_process(record_lever)
                camera.stop_recording()
                camera.stop_preview()
            record_event(datetime.now(), '99')
            save_mouse_variables()
        motor.duty_cycle = LOW_DUTY_CYCLE
        sleep(0.1)
        motor.enable = False
        GPIO.cleanup()
        buzzer.unexport()
        motor.unexport()
        stream.clear()
        camera.close()
        lever.close()


if __name__ == '__main__':
    main()
