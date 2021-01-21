# PiPaw

PiPaw is an automated home-cage system for assessing motor learning and movement kinematics in mice.

# Files

piPaw_amplitude.py - Main script required to run the PiPaw cage.

mice.cfg - This file maps each RFID to a mouse name, and stores a log of water received, rewarded and failed trials, day of testing and testing phase. This should be edited before the start of testing to include the RFIDs for each mouse to be tested.

monitor.sh - Bash script which should be used to run piPaw_amplitude.py. This script will log all output of PiPaw to a file called log.txt and will restart the program should it quit due to an error. This script should be set to run on boot through a crontab task.

classReadEncoder.py - Library to interface with the encoder chip.

pwm.py - Library to create PWM pulses for controlling the motor.
