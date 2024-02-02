import RPi.GPIO as GPIO
import serial

from time import time, sleep

from src.constants import *

RFID_CHARS = ['0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F']

class RFID:
    def __init__(self):
        GPIO.setup(G_RFID, GPIO.IN)
        self.port = serial.Serial("/dev/ttyUSB0", baudrate=9600)
        self.buffer = []
        self.interrupted = False

    def rfid_detected(self):
        """
        Returns True if RFID tag is detected, False otherwise.
        """
        return GPIO.input(G_RFID) == 1

    def get_serial(self):
        """
        Reads the most recently detected 12 character RFID input from the serial port
        and ignores all others in the buffer.

        Port is a serial object.
        Returns a str containing the RFID.
        """
        # Short timeout to ensure that the full serial is detected by the RFID before moving forward
        sleep(0.05)
        # Get the last detected serial input
        while True:
            line = self.port.readline()
            # Read out RFIDs until the last one (if there is more than 1)
            if self.port.inWaiting() < 16:
                break

        # Raise error if nothing was found in the buffer
        if not line:
            raise ValueError("No RFID detected (logical input is high).")

        # Format and return
        rfid = ''
        for c in line:
            if chr(c) in RFID_CHARS:
                rfid += chr(c)
        self.port.flushInput()
        return rfid

    def out_of_range(self):
        if not self.interrupted and self.port.inWaiting() >= 16:
            print("RFID went out of range during trial.")
            self.interrupted = True
            return True
        return False


    def cleanup(self):
        """
        Cleans up GPIO pins on program exit.
        """
        GPIO.cleanup()
