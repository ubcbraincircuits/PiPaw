from src.pwm import PWM
from time import sleep

#To enable the hardware PWM timer on GPIO 18 and 19, edit the config.txt file with the following:
#dtoverlay=pwm-2chan
#The piezo buzzer is connected to GPIO-19 which is the default pwm1 output.

buzzer = PWM(1)
buzzer.export()

def playTone(freq, duration):
    period = int(1000000000/freq)
    duty = int(period/2)
    if buzzer.duty_cycle > period:
        buzzer.duty_cycle = duty
        buzzer.period = period
    else:
        buzzer.period = period
        buzzer.duty_cycle = duty
    buzzer.enable = True
    sleep(duration)
    buzzer.enable = False

try:
    playTone(523.25, 0.4)
    playTone(587.33, 0.4)
    playTone(659.25, 0.4)
    playTone(698.46, 0.4)
    playTone(783.99, 0.4)
    playTone(880, 0.4)
    playTone(987.77, 0.4)
    playTone(1046.5, 0.4)

except OSError as err:
    print("OS Error: {0}".format(err))
    buzzer.unexport()
    
buzzer.unexport()
    



