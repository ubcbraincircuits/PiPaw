import RPi.GPIO as GPIO

from src.constants import *

class Infrared:
    def __init__(self):
        GPIO.setup(G_INFRARED, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def beam_broken(self):
        """
        Returns True if infrared beam is broken, False otherwise.
        """
        return GPIO.input(G_INFRARED) == 0
