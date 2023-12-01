import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

g_SOLENOID = 4

GPIO.setup(g_SOLENOID, GPIO.OUT)

def Solenoid():
    print('Press enter to turn on the solenoid.')
    input()
    GPIO.output(g_SOLENOID, True)
    print('Solenoid is [ON]...')
    print('Press enter to turn off the solenoid.')
    input()
    GPIO.output(g_SOLENOID, False)
    print('Solenoid is [OFF]')


def main():
    print('Hello! This short script will hopefully help us calibrate the water flow through the solenoid, when you no longer need it just press \'CTRL + C\'')
    try:
        while True:
            Solenoid()
    except KeyboardInterrupt:
        GPIO.cleanup()
        print('Goodbye! I hope the water is flowing nicely!')

main()
