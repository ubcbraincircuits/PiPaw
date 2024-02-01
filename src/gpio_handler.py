import RPi.GPIO as GPIO

from .constants import *
from time import sleep


class GPIOHandler:
    def __init__(self):
        self.setup_gpio()

    def setup_gpio(self):
        """
        Sets up the GPIO pins with the required configuration.
        """
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(G_RFID, GPIO.IN)
        GPIO.setup(G_SOLENOID, GPIO.OUT)

    def rfid_detected(self):
        """
        Returns True if RFID tag is detected, False otherwise.
        """
        return GPIO.input(G_RFID) == 1

    def operate_solenoid(self, duration):
        """
        Operates the solenoid for a specified duration.
        """
        GPIO.output(G_SOLENOID, True)
        sleep(duration)
        GPIO.output(G_SOLENOID, False)

    def cleanup(self):
        """
        Cleans up GPIO pins on program exit.
        """
        GPIO.cleanup()
