from src.pwm import PWM
from time import sleep

class Buzzer:

    def __init__(self):
        # PWM1 defaults to GPIO19 which is where the buzzer should be connected
        self.buzzer = PWM(1)
        self.buzzer.export()

    def play_tone(self, freq, n=1, duration=0.2, iti=0.1):
        """
        Plays one or more tones of a specified frequency and duration and inter-tone interval.
        freq is a str indicating either a Hi, Med or Low tone
        n is the number of repetitions (defaults to 1)
        duration is length in seconds of the tone (defaults to 0.2)
        iti is the break between tones in seconds (defaults to 0.1)
        """
        freq_dict = {'Hi': 5000, 'Med': 2500, 'Low': 1000}
        period = int(1000000000/freq_dict[freq])
        duty = int(period/2)

        if self.buzzer.duty_cycle > period:
            self.buzzer.duty_cycle = duty
            self.buzzer.period = period
        else:
            self.buzzer.period = period
            self.buzzer.duty_cycle = duty
        while n > 0:
            self.buzzer.enable = True
            sleep(duration)
            self.buzzer.enable = False
            n-=1
            if n > 0:
                sleep(iti)

    def cleanup(self):
        self.buzzer.unexport()