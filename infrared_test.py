"""
Created July 25 2017 by Cam

Script for testing out functionality of the infrared breakbeam.
"""

import RPi.GPIO as GPIO
import time

g_infrared =21

GPIO.setmode(GPIO.BCM)
GPIO.setup(g_infrared, GPIO.IN, pull_up_down=GPIO.PUD_UP)   #This breakbeam requires pull-up resistor to be activated

if GPIO.input(g_infrared) == 0:
    print("Beam Broken")
    lastState = 0
else:
    print("Solid")
    lastState = 1
while True:
    currentState = GPIO.input(g_infrared)
    if lastState == currentState:
        time.sleep(0.01)
    else:
        if currentState == 0:
            print("Beam Broken")
            lastState = 0
        else:
            print("Solid")
            lastState = 1
