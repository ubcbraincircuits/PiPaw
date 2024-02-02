from time import sleep
import RPi.GPIO as GPIO

from src.constants import *


class Solenoid:
    def __init__(self):
        GPIO.setup(G_SOLENOID, GPIO.OUT)

    def dispense_water(self, t):
        """
        Dispenses water for a specified duration.
        """
        GPIO.output(G_SOLENOID, True)
        sleep(t)
        GPIO.output(G_SOLENOID, False)