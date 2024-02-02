import RPi.GPIO as GPIO
from time import sleep

from src.pwm import PWM

g_motorEnable = 23
g_motorDirection = 24
GPIO.setmode(GPIO.BCM)
GPIO.setup(g_motorEnable, GPIO.OUT)
GPIO.output(g_motorEnable, False)
GPIO.setup(g_motorDirection, GPIO.OUT)
GPIO.output(g_motorDirection, True)
motor = PWM(0)
motor.export()
motor.period = 1000000 #1 million ns = 1000Hz

try:

    input("This script will test the motors response to duty cycle commands from 10 to 90 percent. Press enter when ready.")

    for x in range(1, 10):
        print("Setting PWM duty cycle to %i percent." % (x*10))
        motor.duty_cycle = x*100000
        if x == 1:
            motor.enable = True
            GPIO.output(g_motorEnable, True)
        if x < 9:
            input("Press enter to move to %i percent." % ((x+1)*10))
        else:
            input("Press enter to disable motor and exit script.")
    
    GPIO.output(g_motorEnable, False)
    motor.enable = False
    GPIO.cleanup()
    motor.unexport()

finally:
    motor.enable = False
    GPIO.cleanup()
    motor.unexport()
        
