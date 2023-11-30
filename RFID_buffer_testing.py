import RPi.GPIO as GPIO
import time
import serial

g_RFID = 12 #GPIO pin of the logical input from the RFID reader

GPIO.setmode(GPIO.BCM)
GPIO.setup(g_RFID, GPIO.IN)

port = serial.Serial("/dev/ttyUSB0", baudrate=9600)

while True:    
    if port.inWaiting() > 0:
        buffer = port.inWaiting()
        print(buffer)
        print(port.read(buffer))
    else:
        time.sleep(0.1)