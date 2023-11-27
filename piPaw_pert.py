"""
Version Update: July 2021 (v2.3)
- (2.2): Fixed incomplete Perturbation issue
- (2.3): Fixed incomplete Perturbation issue (while not ns.trial_started)

piPaw lever cage program for assessing forelimb motor learning in the mouse home-cage.
The main criteria for success in this version is the time the lever is held in a reward range.
This testing methodology consists of two training phases followed by an main phase:

    Training:       Lever must be pulled past the threshold position. There are no failed trials, anything past the threshold is successful.
                    The trial ends when the lever returns below threshold.
                    Mouse must perform 200 successful trials in this phase in order to move onto the Main phase
                    During this phase, the mouse also gets a free water drop every 15 minutes just for nose poking (FI Schedule)
    Main:           Lever must be pulled back between 6 and 18 degrees (middle half of the full position range) and held for a certain amount of time
                    The amount of time the lever must be held in this range starts at 0.1 seconds, and increases as the mouse learns to hold for longer
                    If the lever exits the goal range before the required time has passed, the trial is failed
                    Once the mouse has performed n trials at each hold time (adjustable), the program begins checking their success rate with a sliding window of 50 trials
                    If they are over 75% successful in this window of trials, they are advanced to a longer hold time (0.1s higher) to a maximum of 1s
                    There is no maximum hold time limit for the trial as long as the lever is in the central goal range.
                    However, once the lever leaves the goal range (either above or below), the trial is finished and the motor returns slowly to high torque to return lever to start position.

For all testing phases, the mouse initiates trial by nose poking and must leave nose in the port for the duration of the trial.
Mice proceed at their own pace through the task in a group housed environment. Trial timing, outcome, lever position data and video is collected for all trials.
"""

import serial, picamera, classReadEncoder
import multiprocessing as mp
import RPi.GPIO as GPIO
from time import time, sleep, mktime, strptime
import datetime
from datetime import datetime,timedelta
from pwm import PWM
from sys import exit
import random
import gc
from os import path, mkdir
import numpy as np
#GPIO Constants
g_infrared = 21
g_solenoid = 4 #Specific GPIO for this should be between 1-8 as these are pulled High by default on boot, otherwise valve will open
g_motorEnable = 23
g_motorDirection = 24
g_RFID = 16

#Trial constants
n_training = 200 #Number of successful trials in training stage
n_successCheck = 50 #Size of the bin in which trials are checked to determine if animal advances to next hold time
successReq = 0.75 #Success ratio required for increase in hold time
initial_ht = 0.05 #Initial hold time at beginning of main phase
ht_increase = 0.1 #Amount hold time is increased by each time
max_ht = 1.0 #Maximum hold time mice can advance to
t_solenoid = 0.25 #Length of time solenoid opens for entrance rewards and successful trials
t_timeout = 5.0 #Minimum duration between two trials
t_timeout_success = 1.0 #Minimum duration between two successful trials
t_timeout_cancelled = 5.0 #Minimum duration between two cancelled trials
t_timeout_failed = 5.0 #Minimum duration between failed trials
t_IRgrace = 0.2 #This raises the gain a bit on the sensitivity of the IR breakbeam, as it sometimes goes high for a fraction of a second when a small amount of light gets through
t_RFIDgrace = 30.0 #Duration after last RFID signal detection that animal is considered to have left the chamber
t_IRWait = 1.0 #Minimum duration IR breakbeam must be broken for before trial initiates (even if timeout is finished)
t_entranceRewardInterval = 900 #Fixed interval for delivery of entrance reward in seconds (only for training 1 phase)
perturbation_wait = 0.5 #Time to wait before starting the perturbation
pert_list = [-1,-0.5,0,0.5,1,2] # % of max force for perturbation
pert_duration = 1.5 #duration of the force perturbation
max_ht_trials = 100
reqSuccessRate = 0.5
t_LEVgrace = 0.2
#Lever position constants for the small motor (1524 SR) - these assume 11.4 clicks per degree (4096 CPR encoder in 1x quadrature mode)
thresholdPos = 68 #used to be 3 degrees
pos1 = 68 #6 degrees
pos2 = 274 # 24 degrees
drivename = "118D-D10B" #name of the external hard drive for video storage

#Mouse variables
mouse_name = ''
mouseDay = datetime.now().day
dt_lastDetected = datetime.now()
tt_trialStart = time()
tt_mouseGone = time()
dailyWater = 0
rewardPulls = 0
failedPulls = 0
meanHT = 0
medianHT = 0
q3HT = 0
totRewardPulls = 0
totFailedPulls = 0
totDays = 0
entranceRewards = 0
pert_trials = 0
hold_time = 0.0
free_water_remained = 0
ht_trials = 0
phase = 1 # 1 = Training, 2 = Acquisition
pert_force = 0.0
#Booleans
dataLoaded = False #Indicates if data has been loaded for an animal
trialStarted = False #Indicates whether a trial is allowed to be started (i.e. head is in position, motor is at low torque)
trialOngoing = False #Indicates that there is currently a trial ongoing (e.g. the lever is in the reward range, or was at last check)
RFIDInterrupt = False #Indicates that RFID needs to be double checked (either logic signal has gone low or data waiting in buffer)
infraredRestored = False #Indicates that infrared breakbeam has been restored
nosePoke = False #Indicates that the mouse is currently nose poking (i.e. head is in position)

#GPIO Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(g_infrared, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(g_RFID, GPIO.IN)
GPIO.setup(g_solenoid, GPIO.OUT)
GPIO.setup(g_motorEnable, GPIO.OUT)
GPIO.setup(g_motorDirection, GPIO.OUT)
GPIO.output(g_motorDirection, True) #True = CW motor direction

#Motor Constants and Setup
motor = PWM(0) #PWM0 defaults to GPIO18 which is where motor should be connected
motor.export()
motor.period = 1000000 #1 million ns = 1000Hz
hiMotor = 850000 #85% duty cycle
loMotor = 150000 #15% duty cycle
#Buzzer Setup
buzzer = PWM(1) #PWM1 defaults to GPIO19 which is where the buzzer should be connected
buzzer.export()

#Camera Constants and Setup
camera = picamera.PiCamera(resolution=(500, 500), framerate = 90) #Initializing camera (EDITED)
stream = picamera.PiCameraCircularIO(camera, seconds=5, bitrate=10000000) #Initializing stream
stream2 = picamera.PiCameraCircularIO(camera, seconds=5, bitrate=10000000) #Initializing stream
camera.vflip = True
camera.hflip = True

#RFID Constants and Setup
port = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout = 1)
RFID_chars = ['0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F']

#Multiprocessing
manager = mp.Manager()
ns = manager.Namespace()
list_leverPos = manager.list()
ns.trial_started = False
ns.trial_over = False
ns.pos1 = False
ns.pos2 = False
ns.pert = 0
ns.time_in_range = time()
ns.grace_period = False
ns.entered_range = False
ns.pert_force = 0
#Encoder Constants and Setup
CSX = 0 #Pin CS0
CLK = 10000000 #10Mhz clock freq.
BYTEMD = 4 #Bytes that will be sent to you.
samplingRate = 410 #Rate of lever sampling in Hz. 205 gets ~200 Hz rate in practice
lever = classReadEncoder.Encoder(CSX,CLK,BYTEMD)

def recordLeverPos(list_leverPos, ns):
    #global ns
    '''
    Takes in an empty list and appends the current encoder position and time to it at a certain rate
    (based on specified sampling rate). Also checks if success range/fail range has been reached.
    '''
    if phase == 1:
        thresholdPos = 34
    else:
        thresholdPos = 66

    while True:
        t_last = time()
        position_reading = lever.readCounter()
        position_time = 0
        #ns.pert=0
        list_leverPos.append([position_time, position_reading,ns.pert])
        if lever.readCounter() > thresholdPos:
            tt_start = time()
            ns.trial_started = True
            ns.start_time = tt_start
            #Simpler to make seperate loops for phase 1 and 2
            if phase == 1:
                while True:
                    t_last = time()
                    position_reading = lever.readCounter()
                    position_time = time() - tt_start
                    list_leverPos.append([position_time, position_reading])

                    if not ns.pos1 and position_reading > pos1:
                        tt_range = time()
                        ns.pos1 = True
                    if ns.pos1 and position_reading > pos2:
                        ns.time_in_range = time() - tt_range
                        ns.pos2 = True
                        ns.trial_over = True
                        return
                    elif ns.pos1 and position_reading <= pos1:
                        ns.time_in_range = time() - tt_range
                        ns.trial_over = True
                        return
                    elif position_reading <= thresholdPos:
                        ns.time_in_range = 0.0
                        ns.trial_over = True
                        return
                    while(time()-t_last < 1.0/samplingRate):
                        sleep(0.0001)

            elif phase == 2 or ns.pert_force ==0:
                print('NO PERTURB')
                while True:
                    t_last = time()
                    position_reading = lever.readCounter()
                    position_time = time() - tt_start
                    list_leverPos.append([position_time, position_reading, ns.pert])
                    if not ns.pos1 and position_reading > pos1:
                        tt_range = time()
                        ns.pos1 = True
                    if ns.pos1 and position_reading > pos2:
                        ns.time_in_range = time() - tt_range
                        ns.pos2 = True
                        ns.trial_over = True
                        return
                    elif ns.pos1 and position_reading <= pos1:
                        ns.time_in_range = time() - tt_range
                        ns.trial_over = True
                        return
                    elif position_reading <= thresholdPos:
                        ns.time_in_range = 0.0
                        ns.trial_over = True
                        return
                    while(time()-t_last < 1.0/samplingRate):
                        sleep(0.0001)

            elif phase == 3 and ns.pert_force !=0:
                while True:
                    t_last = time()
                    position_reading = lever.readCounter()
                    position_time = time() - tt_start
                    
                    list_leverPos.append([position_time, position_reading, ns.pert])
                    if not ns.pos1 and not ns.entered_range and position_reading > pos1:

                        ns.entered_range = True
                        tt_range = time()
                        ns.pos1 = True
                    #if not ns.pos1 and ns.entered_range and position_reading > pos1:
                        #ns.pos1 = True
                    if ns.pos1 and position_reading > pos2:
                        ns.time_in_range = time() - tt_range
                        if not ns.grace_period and ns.time_in_range > perturbation_wait:
                            ns.grace_period = True
                            tt_grace = time()
                        if ns.grace_period and time() - tt_grace > t_LEVgrace:
                            ns.pos2 = True
                            ns.trial_over = True
                            ns.grace_period = False
                            return
                        if not ns.grace_period and ns.time_in_range < perturbation_wait:
                            ns.pos2 = True
                            ns.trial_over = True
                            ns.grace_period = False
                            return
                    elif ns.pos1 and position_reading <= pos1:
                        ns.time_in_range = time() - tt_range
                        if not ns.grace_period and ns.time_in_range > perturbation_wait:
                            ns.grace_period = True
                            tt_grace = time()
                        if ns.grace_period and time() - tt_grace > t_LEVgrace:
                            ns.trial_over = True
                            return
                    elif position_reading <= thresholdPos:
                        ns.time_in_range = 0.0
                        ns.trial_over = True
                        return
                    while(time()-t_last < 1.0/samplingRate):
                        sleep(0.0001)



        while(time()-t_last < 1.0/samplingRate):
            sleep(0.0001)

def writeLeverPos(t_event, list_leverPos, event):
    """
    Appends the lever position data from one trial into the leverPos file for that animal
    t_event is a datetime object
    list_leverPos is the list written by recordLeverPos
    event is a str containing the event code
    """

    with open('/home/pi/piPaw/data/%s/%s_leverPos.txt' %(mouse_name, mouse_name), 'a') as file:
        initialLine = str(t_event) + '\t' + event + '\n'
        file.write(initialLine)
        first_index = 0
        for x in range(len(list_leverPos)):
            if list_leverPos[x][1] > thresholdPos:
                if x > 100:
                    first_index = x - 100
                else:
                    first_index = 0
                break
        for x in range(first_index, len(list_leverPos)):
            line = list_leverPos[x]
            line_to_write = str(line[0]) + '\t' + str(line[1]) + '\t' + str(line[2]) + '\n'
            file.write(line_to_write)
    print("Lever position saved to file.")


def checkID(tag):
    """
    Checks to see if animal's RFID is in mice.cfg, and if it matches previously detected mouse
    tag is animal RFID
    """
    with open('/home/pi/piPaw/mice.cfg', 'r') as file:
        miceLines = file.readlines()

    for mouseLine in miceLines:
        mouseLine = mouseLine.replace('\n', '')
        mouseLine = mouseLine.split('\t')
        if mouseLine[0] == tag:
            #If data is loaded already, we're checking to see if the mouse is the same as was previously detected
            if dataLoaded:
                if mouseLine[1] != mouse_name:
                    print('Mouse has changed! %s:%s ' % (mouseLine[1], mouseLine[0]) + 'entered the chamber (' + str(datetime.now()) + ').')
                    return True # Identifies mouse change
                else:
                    print("Mouse has not changed.")
                    return False
            #If data is not loaded, we are just checking to make sure mouse name is in mice.cfg, don't need to return anything
            print('%s:%s ' % (mouseLine[1], mouseLine[0]) + 'entered the chamber (' + str(datetime.now()) + ').')
            return

    #If checkID gets through the full mice.cfg file without finding the mouse name, should raise a value error
    print("Mouse ID = " + tag)
    raise ValueError("Mouse name not found in mice.cfg.")

def getSerial():
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

def recordData(timestamp, event, ht, dt=0.0, pert_force=0.0):
    """
    Records trial and entrance data to relevant file with the following event codes:
        00: Mouse entered
        99: Mouse exited
        01: Mouse given entrance reward
        02: Successful trial (training phase)
        03: Successful trial - exited forwards
        04: Successful trial - exited backwards
        51: Cancelled trial - nose removed from port mid-trial (note that this does not count as a failed trial for success rate purposesand lever position/video are not recorded)
        52: Cancelled trial - did not reach success range (note that this does not count as a failed trial for success rate purposes and lever position/video are not recorded)
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
    with open('/home/pi/piPaw/data/%s/%s_data.txt' %(mouse_name, mouse_name), 'a') as file:
        data = str(timestamp) + '\t' + event + '\t' + str(ht) + '\t' + str(round(dt, 4)) + '\t' + str(pert_force) + '\n'
        file.write(data)

def updateHoldTime(mouse_name,ht,phase):
    if phase == 1 or phase == 2 or phase == 3:
        ht_list=[]
        f_list=[]
        success_codes = ['02','03', '04']
        fail_codes = ['53','54']
        with open('/home/pi/piPaw/data/%s/%s_data.txt' %(mouse_name, mouse_name) , 'r') as file:
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
            return ht,phase,0,0,0
        medianHT = round(np.percentile(ht_list,50),2)
        q3HT = round(np.percentile(ht_list + f_list,75),2)
        meanHT = round(np.mean(ht_list + f_list),2)
        if phase ==1:
            return 0,1,medianHT,meanHT, q3HT

        if len(ht_list)>100:
            if ht == max_ht and len(ht_list)/(len(ht_list) + len(f_list)) > reqSuccessRate:
                phase = 3
                hold_time = max_ht
                print("Phase 3: Perturbation started")
            elif q3HT > max_ht:
                hold_time = max_ht
                print("Maximum Hold time reached.")
            elif ht > q3HT:
                hold_time = ht
                print('75th percentile (' +str(q3HT)+ ') is less than current hold time. The holdtime is not changed.')
            elif len(ht_list)/(len(ht_list) + len(f_list)) > 0.3:
                if q3HT > ht + 0.3:
                    hold_time = ht + 0.1
                    print('75th percentile (' +str(q3HT)+ ') is more than 0.3s longer than current HT. The holdtime is increased by 0.1s.')
                else:
                    hold_time = q3HT
                    print('Hold time set to '+ str(hold_time))
            else:
                holdtime = ht
            return hold_time,phase,medianHT,meanHT,q3HT
        else:
            print("Minimum daily water consumption was less than 1 mL. The Hold time is not changed.")
            return ht,phase,medianHT,meanHT, q3HT



def updateMouseVars(tag):
    """
    Takes an RFID tag and updates mouse variables for the current mouse. Re-initializes daily variables if it's past midnight.
    """
    global mouse_name, dailyWater, rewardPulls, mouseDay, failedPulls, totRewardPulls, totFailedPulls, totDays, entranceRewards, hold_time, ht_trials, phase,pert_trials,free_water_remained, meanHT,medianHT,q3HT
    with open('/home/pi/piPaw/mice.cfg', 'r') as file:
        miceLines = file.readlines()

    for mouseLine in miceLines:
        mouseLine = mouseLine.replace('\n', '')
        mouseLine = mouseLine.split('\t')

        if mouseLine[0] == tag:
            mouse_name = mouseLine[1]
            mouseDay = int(mouseLine[2])
            dailyWater = int(mouseLine[3])
            rewardPulls = int(mouseLine[4])
            failedPulls = int(mouseLine[5])
            totRewardPulls = int(mouseLine[6])
            totFailedPulls = int(mouseLine[7])
            totDays = int(mouseLine[8])
            entranceRewards = int(mouseLine[9])
            hold_time = float(mouseLine[10])
            ht_trials = int(mouseLine[11])
            pert_trials = int(mouseLine[12])
            phase = int(mouseLine[13])
            free_water_remained = int(mouseLine[14])
            now = datetime.now()
            if now.day != mouseDay:
                print("Rollover time reached. Resetting daily variables...")
                old_ht = hold_time
                if phase == 1 or phase == 2 or phase == 3:
                    hold_time,phase,medianHT,meanHT,q3HT = updateHoldTime(mouse_name,hold_time,phase)
                    htt = ht_trials
                    ht_trials = 0
                free_water_remained = max(0 , 100 - dailyWater)
                recordDailyReport(rewardPulls,failedPulls,meanHT,medianHT,q3HT,free_water_remained,phase,old_ht)
                dailyWater = 0
                rewardPulls = 0
                failedPulls = 0
                totDays += 1

                mouseDay = now.day
            return

def recordDailyReport(rewardPulls,failedPulls,meanHT,medianHT,q3HT,free_water_remained,phase,old_ht):
    """
    Record the daily report.
    """
    date = datetime.today() - timedelta(days=1)
    with open('/home/pi/piPaw/data/%s/%s_dailyreport.txt' %(mouse_name, mouse_name), 'a') as file:
        data = str(date.date()) + '\t' + str(rewardPulls) + '\t' + str(failedPulls) + '\t' + str(meanHT) + '\t' + str(medianHT) + '\t' + str(q3HT) +'\t' + str(old_ht) + '\t' + str(free_water_remained) + '\t' + str(phase) + '\n'
        file.write(data)

def saveMouseVars():
    """
    Saves mouse variables and writes to file when they leave the chamber.
    """
    with open('/home/pi/piPaw/mice.cfg', 'r') as file:
        data = file.readlines()

    for j in range(len(data)):
        data[j] = data[j].replace('\n', '')
        data[j] = data[j].split('\t')

        if data[j][1] == mouse_name:
            data[j][2] = mouseDay
            data[j][3] = dailyWater
            data[j][4] = rewardPulls
            data[j][5] = failedPulls
            data[j][6] = totRewardPulls
            data[j][7] = totFailedPulls
            data[j][8] = totDays
            data[j][9] = entranceRewards
            data[j][10] = hold_time
            data[j][11] = ht_trials
            data[j][12] = pert_trials
            data[j][13] = phase
            data[j][14] = free_water_remained

    with open('/home/pi/piPaw/mice.cfg', 'w') as file:
        for j in range(len(data)):
            newLine = ''
            for i in range(len(data[j])):
                newLine += str(data[j][i]) + ('\n' if i == (len(data[j])-1) else '\t')
            file.write(newLine)

def killLeverPosProc(p):
    """
    Terminates recordLeverPos.
    """
    while True:
        p.terminate()
        if p.is_alive() == False:
            break

def timeOut(aht,rht):
    timeout = 5 - 4*(aht/(rht+0.000001))
    if timeout < 1:
        timeout = 0.5
    return timeout


def dispenseWater(t):
    """
    Opens the solenoid for t length of time and then closes it.
    """
    GPIO.output(g_solenoid, True)
    sleep(t)
    GPIO.output(g_solenoid, False)

def motorRamp():
    """
    Ramps the motor up from low to high strength over 0.45s. Motor must already be enabled.
    """
    for x in range (11, 90):
        motor.duty_cycle = x*10000
        sleep(0.005)
    motor.duty_cycle = hiMotor

def playTone(freq, n=1, duration=0.2, iti=0.1):
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


def writeVideo(timestamp, code):
    print('WRITING VIDEO')
    """
    Writes video from stream either on an external drive or the internal storage; timestamp is the datetime of the trial, code is the trial code.
    """
    date = str(timestamp.month) + '-' + str(timestamp.day) + '-' + str(timestamp.year) + '-' +str (timestamp.hour) + str(timestamp.minute) + str(timestamp.second)
    if stream.tell():
        with open('/home/pi/piPaw/data/%s/Videos/%s_%s_%s_BEFORE.h264' % (mouse_name, mouse_name, code, date), 'wb') as output:
            stream.copy_to(output)
    if stream2.tell():
        with open('/home/pi/piPaw/data/%s/Videos/%s_%s_%s_AFTER.h264' % (mouse_name, mouse_name, code, date), 'wb') as output2:
            stream2.copy_to(output2)
    stream.clear()
    stream2.clear()


def checkEntranceReward():
    """
    Checks if animal has reached interval to get another entrance reward (if in training 1)
    """

    with open('/home/pi/piPaw/data/%s/%s_data.txt' %(mouse_name, mouse_name), 'r') as file:
        events = file.readlines()

    for x in range(len(events)-1, 0, -1):
        event = events[x]
        event = event.replace('\n', '')
        event = event.split('\t')

        if event[1] == '01':
            last_drop_time = mktime(strptime(event[0], "%Y-%m-%d %H:%M:%S.%f"))
            interval = time() - last_drop_time
            if interval > t_entranceRewardInterval:
                print("%i seconds since last entrance reward given. Delivering entrance reward." %(int(interval)))
                return True
            print("Insufficient time has passed since last entrance reward (%i seconds)" %(int(interval)))
            return False
    print("Delivering first entrance reward.")
    return True

"""
def checkSuccess():

#    Checks if animal has met x% success criteria over last y trials (x = successReq*100, y = n_checkSuccess).
#    Returns True if criteria is met, otherwise returns False

    fail_codes = ['53', '54']
    success_codes = ['03', '04']
    successes = 0
    total = 0

    with open('/home/pi/piPaw/data/%s/%s_data.txt' %(mouse_name, mouse_name), 'r') as file:
        events = file.readlines()

    for x in range(len(events)-1, 0, -1):
        event = events[x]
        event = event.replace('\n', '')
        event = event.split('\t')

        if event[1] in success_codes:
            successes += 1
            total += 1
        elif event[1] in fail_codes:
            total += 1

        if total == n_successCheck:
            break

    if successes / total >= successReq:
        print("%s has met criteria for increased hold time (%i successes out of %i total)." %(mouse_name, successes, total))
        return True
    else:
        print("%s has not met criteria for increased hold time (%i successes out of %i total)." %(mouse_name, successes, total))
        return False
"""
def endTrial(datetimeStart, trialTime, event, outcome, pert_force=0.0):
    """
    Ends a trial when success or fail condition has been met. Returns lever to start position, records data and updates variables.
    datetimeStart is a datetime object
    trialTime is a float indicating the length of the trial
    event is a string indicating the event code
    outcome is a string indicating if trial was successful, failed or cancelled
    """
    global tt_trialStart, tt_mouseGone, trialStarted, trialOngoing, dailyWater, rewardPulls, failedPulls, totRewardPulls, totFailedPulls, ns, RFIDInterrupt, dt_lastDetected, hold_time, ht_trials,max_ht_trials, pert_trials, phase

    tt_finished = time()
    motorRamp() #Returning motor to high torque setting
    recordData(datetimeStart, event, hold_time, trialTime, pert_force) #Recording event
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
        trialStarted = False
        trialOngoing = False

    #Following section should only be executed if trial was not cancelled
    elif not outcome == 'cancelled':
        #Playing tones, delivering water and updating mouse variables
        if outcome == 'success':
            playTone('Hi')
            rewardPulls += 1
            totRewardPulls += 1
            dailyWater += 1
            print("Successful trial: code %s. %s has %i successful trial(s) today, %i in total (" % (event, mouse_name, rewardPulls, totRewardPulls) + str(datetimeStart) + ").")
            dispenseWater(t_solenoid)
        elif outcome == 'failed':
            playTone('Low',duration=0.5)
            failedPulls += 1
            totFailedPulls += 1
            print("Failed trial: code %s. %s has %i failed trial(s) today, %i in total (" % (event, mouse_name, failedPulls, totFailedPulls) + str(datetimeStart) + ").")

        #Stopping video
        camera.stop_preview()
        camera.stop_recording()

        #Recording video and lever position
        writeVideo(datetimeStart, event)
        writeLeverPos(datetimeStart, list_leverPos, event)
        stream2.clear()
        camera.start_recording(stream2, format='h264')

        #Checking if phase has changed
        if phase == 1:
            if totRewardPulls >= n_training:
                print("%s has reached %i successful trials. Changing to main stage." %(mouse_name, totRewardPulls))
                hold_time,*_ = updateHoldTime(mouse_name,initial_ht,phase)
                phase = 2
                recordData(datetime.now(), '66', hold_time) #66 is switching to main stage code
        elif phase == 2:
            ht_trials += 1
#            if hold_time < max_ht and ht_trials >= n_successCheck:
            print("%s has %i trials at %f second hold time. " %(mouse_name, ht_trials, hold_time))
        elif phase == 3:
            ht_trials += 1
            if pert_force != 0:
                pert_trials += 1
                print("%s has %i perturbation trials. " %(mouse_name, pert_trials))


    #Updating booleans and timing variables, and recording data
    tt_trialStart = time()
    stream.clear()
    ns.trial_started = False
    ns.trial_over = False
    ns.pos1 = False
    ns.pos2 = False
    trialStarted = False
    trialOngoing = False
    ns.entered_range = False
    ns.grace_period = False

    print("Recorded trial data and reset variables in %f seconds." %(time()-tt_finished))

    #During endTrial, it's possible that the mouse may have changed - the RFIDInterrupt boolean will trigger a re-check of the RFID if there is data in the serial buffer
    if not RFIDInterrupt and port.inWaiting() >= 16:
        print("RFID went out of range during trial.")
        RFIDInterrupt = True
        tt_mouseGone = time()
        dt_lastDetected = datetime.now()


def perturbation(pert_force):
    global ns
    gc.collect ()
    perturbation_duty = int(loMotor + (abs(pert_force))*loMotor)
    trial_started = False
    perturbation_started = False
    perturbation_finished = False
    event = None
    #ns.pert = 0

    while True: # Checking IR signal for perturbation
        if not ns.trial_over:
            print("Perturbation trial initiated. The force is ", pert_force * 100, "% of maximum, latency to perturbation is ", perturbation_wait, " s." )
            tt_perturbation = ns.start_time
            trial_started = True
            while not ns.trial_over:
                position_reading = lever.readCounter()
                if (position_reading < pos1 or position_reading > pos2) and not perturbation_started:
                    trial_started = False
                elif not perturbation_started and not perturbation_finished and time() - tt_perturbation >= perturbation_wait:
                    if pert_force<0:
                        GPIO.output(g_motorDirection, False)
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
                elif perturbation_started and time() - tt_perturbation >= pert_duration:
                    motor.duty_cycle = loMotor
                    ns.pert = 0
                    perturbation_started = False
                    perturbation_finished = True
                    GPIO.output(g_motorDirection, True)
                    print("Perturbation finished.")
                sleep(0.001)

            if perturbation_started:
                #motor.duty_cycle = loMotor
                GPIO.output(g_motorDirection, True)
                print("Lever went out of range during perturbation. (Code 55)")
                perturbation_started = False
                ns.pert = 0
                event = 55
            elif not perturbation_finished:
                print("Lever went out of range before perturbation started. (Code 56)")
                #motor.duty_cycle = loMotor
                GPIO.output(g_motorDirection, True)
                perturbation_started = False
                ns.pert = 0
                event = 56
            GPIO.output(g_motorDirection, True)
            return
        if ns.trial_over:
            print('Perturbation Cancelled!')
            return
        sleep(0.001)




def main():

    global dailyWater, entranceRewards, tt_trialStart, tt_mouseGone, dataLoaded, trialStarted, trialOngoing, list_leverPos, ns, mouseChange, RFIDInterrupt, nosePoke, infraredRestored, dt_lastDetected
    global t_timeout, t_timeout_cancelled, t_timeout_failed, t_timeout_success, pert_duration, perturbation_wait, pert_list, pert_trials, pert_force , free_water_remained
    try:
        #Following loop checks if a mouse is already in the chamber when the program first starts - will activate lever at high torque and sleep until they leave
        if GPIO.input(g_infrared) == 0 or GPIO.input(g_RFID) == True:
            motor.duty_cycle = hiMotor
            motor.enable = True
            GPIO.output(g_motorEnable, True)
            print("Mouse in chamber during program start... waiting for it to leave.")
            tt_chamberClear = time()
            while GPIO.input(g_infrared) == 0 or GPIO.input(g_RFID) == True:
                sleep(0.1)
            motor.duty_cycle = loMotor
            sleep(0.1)
            GPIO.output(g_motorEnable, False)
            motor.enable = False
            print("Chamber cleared in %f seconds." %(time()-tt_chamberClear))

        #Need to make sure the lever is in starting position before the encoder count clears
        motor.duty_cycle = loMotor
        motor.enable = True
        GPIO.output(g_motorEnable, True)
        sleep(0.5)
        lever.clearCounter()
        GPIO.output(g_motorEnable, False)
        motor.enable = False

        print("Done Init. Waiting for mouse.")
        while True:
            #The following two conditionals are checked at the beginning of every main body loop to determine whether a mouse is in the chamber and what it's identity is
            if GPIO.input(g_RFID) == True:
                #Following is executed if an RFID has come into range for the first time since the grace period last expired
                if not dataLoaded:
                    tagID = getSerial() #Reads off the last detected RFID in the serial buffer - raises an error if none detected
                    checkID(tagID) #Raises an error if mouse is not in mice.cfg
                    motor.duty_cycle = hiMotor
                    motor.enable = True
                    GPIO.output(g_motorEnable, True)
                    updateMouseVars(tagID)
                    recordData(datetime.now(), '00', hold_time) #00 is the mouse entrance code
                    dataLoaded = True
                    #camera.start_preview()
                    camera.start_recording(stream, format='h264')
                    camera.start_preview()
                    print("Data loaded and motor enabled. Waiting for nose poke...")
                #Following executes if RFID has come back into range within grace period - need to check if it's the same mouse
                elif RFIDInterrupt:
                    RFIDInterrupt = False
                    print("RFID signal restored. Double checking animal identification.")
                    tagID = getSerial()
                    if checkID(tagID): #Returns True if last ID in the buffer matches a different mouse from the currently stored mouse_name
                        #Save data for previous mouse:
                        camera.stop_recording()
                        stream.clear()
                        recordData(dt_lastDetected, '99', hold_time) #99 is the mouse exit code
                        t_timeout = t_timeout_cancelled
                        saveMouseVars()
                        #Reset booleans and load data for new mouse:
                        nosePoke = False
                        updateMouseVars(tagID)
                        recordData(datetime.now(), '00', hold_time) #00 is the mouse entrance code
                        print("Waiting for nose poke...")
                        camera.start_recording(stream, format='h264')
                        camera.start_preview()

            elif GPIO.input(g_RFID) == False:
                #Following evaluates True when a previously detected mouse goes out of range:
                if dataLoaded and not RFIDInterrupt:
                    print("RFID signal for %s went out of range." %(mouse_name))
                    RFIDInterrupt = True
                    tt_mouseGone = time() #Begins countdown to end of RFID detection grace period
                    dt_lastDetected = datetime.now() #Time that mouse went out of range (used for exit time recording)
                #Following evaluates once grace period for mouse detection is over and mouse is assumed to have left the chamber:
                elif RFIDInterrupt and time() - tt_mouseGone > t_RFIDgrace and not nosePoke:
                    print("%s not detected for more than %i seconds. Assuming chamber is empty (%s)." %(mouse_name, t_RFIDgrace, str(datetime.now())))
                    #Recording exit, saving variables and clearing encoder count:
                    recordData(dt_lastDetected, '99', hold_time) #99 is the mouse exit code
                    t_timeout = t_timeout_cancelled
                    saveMouseVars()
                    camera.stop_recording()
                    camera.stop_preview()
                    stream.clear()
                    stream2.clear()
                    tt_leverBackInPos = time()
                    while lever.readCounter() > 6: #6 is equivalent to ~1/2 degree
                        if (time() - tt_leverBackInPos) > 2.0:
                            print("Possible Error, lever counter value prior to clearing is %i." %(lever.readCounter()))
                            break
                    lever.clearCounter()
                    #Turn off motor and reset booleans
                    motor.duty_cycle = loMotor
                    sleep(0.1)
                    GPIO.output(g_motorEnable, False)
                    motor.enable = False
                    dataLoaded = False
                    RFIDInterrupt = False
                    nosePoke = False
                else: #This should be the only statement executed any time there is no mouse in the chamber and the grace period has expired
                    sleep(0.01)

            if not nosePoke:
                if GPIO.input(g_infrared) == 0: #Infrared is broken (e.g. a nose poke has occurred)
                    if not dataLoaded: #Nose poke detected before RFID was identified
                        print("Infrared broken before RFID was detected. Waiting until ID can be determined.")
                        tt_RFIDDetect = time()
                        while GPIO.input(g_RFID) == False:
                            if time() - tt_RFIDDetect > 10:
                                raise ValueError("Infrared broken but RFID could not be detected within 10 seconds.")
                            sleep(0.01)
                        continue #Will loop back to beginning of loop and detect animal
                    print("Infrared broken. Waiting to begin trial.")
                    nosePoke = True
                    tt_IRBroken = time()
                    #Delivers a free water drop for the the nose poke if required
                    if phase == 1 or free_water_remained != 0:
                        if checkEntranceReward():
                            dispenseWater(t_solenoid)
                            playTone('Hi') #We want to create association between high tone and reward delivery, even for free water
                            dailyWater += 1
                            entranceRewards += 1
                            camera.stop_recording()
                            #stream.clear()
                            if phase != 1:
                                free_water_remained -= 1
                            recordData(datetime.now(), '01', hold_time) #01 is nose poke reward code
                            tt_trialStart = time() #If mouse given nose poke drop, wait 5 seconds until the first trial (otherwise can start immediately)
                            writeVideo(datetime.now(),'01')
                            camera.start_recording(stream2,'h264')
            while nosePoke:
                if GPIO.input(g_infrared) == 1: #Animal has removed head from position
                    if not infraredRestored:
                        tt_trialCancel = time()
                        infraredRestored = True
                    elif time() - tt_trialCancel > t_IRgrace:
                        if trialStarted and not trialOngoing:
                            killLeverPosProc(p)
                            camera.stop_preview()
                            camera.stop_recording()
                            stream2.clear()
                            writeVideo(datetime.now(),'50')
                            camera.start_recording(stream2,'h264')
                            motor.duty_cycle = hiMotor
                            trialStarted = False
                            print("Head removed from position before trial was initiated.")
                        elif trialOngoing:
                            killLeverPosProc(p)
                            print("Head removed before trial could be completed. Trial cancelled (Code 51).")
                            endTrial(t_trialStart, 0.0, '51', 'cancelled') #51 is trial code for head being removed mid-trial
                            t_timeout = t_timeout_cancelled
                        else:
                            print("Head removed from position .")
                        nosePoke = False
                        infraredRestored = False
                        break
                elif GPIO.input(g_infrared) == 0 and infraredRestored:
                    infraredRestored = False
                    tt_IRBroken = time()

                if not trialStarted: #Enough time has past since last trial and animal can start the trial
                    
                    if (time() - tt_trialStart) > t_timeout and (time() - tt_IRBroken) > t_IRWait:
                        trialStarted = True
                        pert_force = random.choice(pert_list)
                        ns.pert_force = pert_force
                        print("Timeout is up, motor switched to low torque.")
                        camera.start_preview()
                        stream2.clear()
                        camera.split_recording(stream2)
                        list_leverPos = manager.list()
                        p = mp.Process(target=recordLeverPos, args=(list_leverPos, ns))
                        p.start()
                        motor.duty_cycle = loMotor
                        playTone('Med')

                elif ns.trial_started and not trialOngoing: #Animal has pulled the lever past threshold
                    t_trialStart = datetime.now()
                    trialOngoing = True
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

                elif trialOngoing and ns.trial_over:
                    killLeverPosProc(p)
                    t_timeout = timeOut(ns.time_in_range,hold_time)
                    if phase == 1:
                        endTrial(t_trialStart, ns.time_in_range, '02', 'success',pert_force=0.0) #02 is success code for Training
                        # t_timeout = t_timeout_success
                    elif phase == 2 or phase == 3:
                        if ns.pos1:
                            if ns.pos2: #Lever exited range towards back of range
                                if ns.time_in_range >= hold_time:
                                    endTrial(t_trialStart, ns.time_in_range, '04', 'success',pert_force) #04 is success code for Acquisition (exited towards back of range)
                                    # t_timeout = t_timeout_success
                                else:
                                    endTrial(t_trialStart, ns.time_in_range, '54', 'failed',pert_force) #54 is fail code for Acquisition (exited towards back of range)
                                    # t_timeout = t_timeout_failed
                            else: #Lever exited range towards front of range
                                if ns.time_in_range >= hold_time:
                                    endTrial(t_trialStart, ns.time_in_range, '03', 'success',pert_force) #03 is success code for Acquisition (exited towards front of range)
                                    # t_timeout = t_timeout_success
                                else:
                                    endTrial(t_trialStart, ns.time_in_range, '53', 'failed',pert_force) #53 is fail code for Acquisition (exited towards front of range)
                                    # t_timeout = t_timeout_failed
                        else: #Lever did not reach goal range - in this case trial is cancelled and recorded in mouse file, but lever position not saved etc.
                            print("Lever did not reach goal range. Trial cancelled (Code 52).")
                            endTrial(t_trialStart, 0.0, '52', 'cancelled') #52 is code for trial did not reach goal range (trial cancelled)
                            t_timeout = t_timeout_cancelled
                    else:
                        t_timeout = t_timeout_cancelled
                        raise ValueError("Testing phase not found.")
                    ns.pos1 = False
                    ns.pos2 = False
                    ns.entered_range = False
                    ns.grace_period = False
                    break

            #Mouse pulls the lever past threshold at any time outside of a trial
            if not trialStarted and lever.readCounter() > thresholdPos:
                if not dataLoaded:
                    playTone('Low',duration=0.5)
                    raise ValueError("Lever moved past threshold before any RFID detected.")
                else:
                    print("Lever moved past threshold during timeout or while head out of position.")
                    tt_backInStartPos = time()
                    while lever.readCounter() > thresholdPos:
                        sleep(0.01)
                    print("Lever returned to threshold range in %f seconds. Restarting time until trial can begin." % (time()-tt_backInStartPos))
                    tt_trialStart = time()

    except KeyboardInterrupt:
        print('Closing program...')

    except ValueError as err:
        print("Value Error: {0}".format(err))
        print("Exit time: "  + str(datetime.now()))
        exit(1)  #Will exit with an error message

    finally:
        if dataLoaded:
            if trialStarted:
                killLeverPosProc(p)
            recordData(datetime.now(), '99', hold_time) #99 is the mouse exit code
            saveMouseVars()
            print("Saving mouse data before closing.")
            t_timeout = t_timeout_cancelled
        motor.duty_cycle = loMotor
        sleep(0.1)
        motor.enable = False
        GPIO.cleanup()
        buzzer.unexport()
        motor.unexport()
        stream.clear()
        camera.close()
        lever.close()


main()
