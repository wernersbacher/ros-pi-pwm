from rpi_hardware_pwm import HardwarePWM, HardwarePWMException
import time

print("Load PWM mod")
pwm = HardwarePWM(pwm_channel=0, hz=50)

print("start")
pwm.start(9)

print("Sleep")
time.sleep(10)