from time import sleep
import RPi.GPIO as GPIO

from src.pwm import PWM
from src.constants import *

ONE_MILLION_NS = 1000000
HIGH_MOTOR = int(ONE_MILLION_NS * 0.85) # 85% duty cycle
LOW_MOTOR = int(ONE_MILLION_NS * 0.15) # 15% duty cycle

class Motor:
    def __init__(self):
        self.gpio_setup()

        # PWM 0 defaults to GPIO18 which is where the motor should be connected
        self.motor = PWM(0)
        self.motor.export()
        self.motor.period = ONE_MILLION_NS # 1 million ns = 1000 Hz
        
        # Start Position
        self.motor.duty_cycle = HIGH_MOTOR
        self.motor.enable = True
        GPIO.output(G_MOTOR_ENABLE, True)
        sleep(0.5)

    def gpio_setup(self):
        """
        Sets up the GPIO pins with the required configuration.
        """
        GPIO.setup(G_MOTOR_ENABLE, GPIO.OUT)
        GPIO.setup(G_MOTOR_DIRECTION, GPIO.OUT)

        # Set default states
        GPIO.output(G_MOTOR_DIRECTION, True)  # True = Clockwise motor direction

    def enable(self, enable=True):
        """
        Enables or disables the motor.
        """
        self.motor.enable = enable
        GPIO.output(G_MOTOR_ENABLE, enable)

    def set_high_duty_cycle(self):
        self.motor.duty_cycle = HIGH_MOTOR

    def set_low_duty_cycle(self):
        self.motor.duty_cycle = LOW_MOTOR

    def set_direction(self, clockwise=True):
        """
        Sets the motor direction.
        """
        GPIO.output(G_MOTOR_DIRECTION, clockwise)

    def ramp(self):
        """
        Ramps the motor up from low to high strength over 0.45s. Motor must already be enabled.
        """
        for x in range (11, 90):
            self.motor.duty_cycle = x*10000
            sleep(0.005)
        self.motor.duty_cycle = HIGH_MOTOR
        
    def disable(self):
        GPIO.output(G_MOTOR_ENABLE, False)
        self.motor.enable = False
            

    def cleanup(self):
        self.motor.duty_cycle = LOW_MOTOR
        sleep(0.1)
        self.motor.enable = False
        GPIO.cleanup()
        self.motor.unexport()
