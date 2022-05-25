#!/usr/bin/python3

import os
import os.path

import rospy
import math
#from std_msgs.msg import String
from ros_pi_pwm.msg import PWMArray, PWM
#from rpi_hardware_pwm import HardwarePWM, HardwarePWMException


class HardwarePWMException(Exception):
    pass

class HardwarePWM:

    _duty_cycle: float
    _hz: float
    chippath: str = "/sys/class/pwm/pwmchip0"

    def __init__(self, pwm_channel: int, hz: float) -> None:

        if pwm_channel not in {0, 1}:
            raise HardwarePWMException("Only channel 0 and 1 are available on the Rpi.")

        self.pwm_channel = pwm_channel
        self.pwm_dir = f"{self.chippath}/pwm{self.pwm_channel}"
        self._duty_cycle = 0

        if not self.is_overlay_loaded():
            raise HardwarePWMException(
                "Need to add 'dtoverlay=pwm-2chan' to /boot/config.txt and reboot"
            )
        if not self.is_export_writable():
            raise HardwarePWMException(f"Need write access to files in '{self.chippath}'")
        if not self.does_pwmX_exists():
            print("pwm does not exist yet, creating.")
            self.create_pwmX()

        while True:
            try:
                self.change_frequency(hz)
                break
            except PermissionError:
                continue


    def is_overlay_loaded(self) -> bool:
        return os.path.isdir(self.chippath)

    def is_export_writable(self) -> bool:
        return os.access(os.path.join(self.chippath, "export"), os.W_OK)

    def does_pwmX_exists(self) -> bool:
        return os.path.isdir(self.pwm_dir)

    def echo(self, message: int, file: str) -> None:
        
        rospy.loginfo(f"{message, file}")
        print(f"{message, file}")
        with open(file, "w") as f:
            f.write(f"{message}\n")

    def create_pwmX(self) -> None:
        print("Creating {pwm_channel}")
        self.echo(self.pwm_channel, os.path.join(self.chippath, "export"))
        print(f"creating success? {self.does_pwmX_exists()}")

    def start(self, initial_duty_cycle: float) -> None:
        self.change_duty_cycle(initial_duty_cycle)
        self.echo(1, os.path.join(self.pwm_dir, "enable"))

    def stop(self) -> None:
        self.change_duty_cycle(0)
        self.echo(0, os.path.join(self.pwm_dir, "enable"))

    def change_duty_cycle(self, duty_cycle: float) -> None:
        """
        a value between 0 and 100
        0 represents always low.
        100 represents always high.
        """
        if not (0 <= duty_cycle <= 100):
            raise HardwarePWMException("Duty cycle must be between 0 and 100 (inclusive).")
        self._duty_cycle = duty_cycle
        per = 1 / float(self._hz)
        per *= 1000  # now in milliseconds
        per *= 1_000_000  # now in nanoseconds
        dc = int(per * duty_cycle / 100)
        self.echo(dc, os.path.join(self.pwm_dir, "duty_cycle"))

    def change_frequency(self, hz: float) -> None:
        if hz < 0.1:
            raise HardwarePWMException("Frequency can't be lower than 0.1 on the Rpi.")

        self._hz = hz

        # we first have to change duty cycle, since https://stackoverflow.com/a/23050835/1895939
        original_duty_cycle = self._duty_cycle
        if self._duty_cycle:
            self.change_duty_cycle(0)

        per = 1 / float(self._hz)
        per *= 1000  # now in milliseconds
        per *= 1_000_000  # now in nanoseconds
        self.echo(int(per), os.path.join(self.pwm_dir, "period"))

        self.change_duty_cycle(original_duty_cycle)




import RPi.GPIO as GPIO

P_SERVO = 12  # adapt to your wiring
P_ENGINE = 33  # adapt to your wiring
fPWM = 50

"""
TODO: Frequenz ändern bei softwarepwm, oder anderes modul nehmen
"""

class PWMSoftware:

    def __init__(self):
        self.pwm_steering = None
        self.pwm_engine = None
        self.setup()

    def setup(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(P_SERVO, GPIO.OUT)
        GPIO.setup(P_ENGINE, GPIO.OUT)
        self.pwm_steering = GPIO.PWM(P_SERVO, fPWM)
        self.pwm_engine = GPIO.PWM(P_ENGINE, fPWM)
        self.pwm_steering.start(7.5)
        self.pwm_engine.start(7.0)


    def update(self, pwm_num, freq, duty):
        if pwm_num == 1:
            rospy.loginfo(f"Setting engine pwm to {duty}%")
            pwm_module = self.pwm_engine
        else:
            pwm_module = self.pwm_steering

        try:
            pwm_module.ChangeDutyCycle(duty)
        except HardwarePWMException as e:
            rospy.logerr("Wrong Hardware values were given!")
            rospy.logerr(str(e))

    def stop(self):
        time.sleep(1)
        self.pwm_steering.stop()
        self.pwm_engine.stop()

        GPIO.cleanup()


class PWMModule:
    """ represents one pwm module"""

    def __init__(self, pwm_channel):
        self.current_duty = 0
        self.current_freq = 0
        self.stopped = True

        self.pwm_channel = pwm_channel
        self.pwm = None

    def start(self):
        self.pwm = HardwarePWM(pwm_channel=self.pwm_channel, hz=self.current_freq)
        self.pwm.start(self.current_duty)

    def update(self, freq, duty):

        is_new_freq = not math.isclose(freq, self.current_freq)

        self.current_duty = duty
        # if new frequencies differs, update the value, but don't apply yet
        if is_new_freq:
            self.current_freq = freq

        if self.stopped:
            self.start()
            self.stopped = False
        else:
            self.pwm.change_duty_cycle(duty)
            if is_new_freq:
                self.pwm.change_frequency(freq)

    def stop(self):
        self.stopped = True
        self.pwm = None


class PWMController:
    """ controls both rpi pwms"""

    def __init__(self):
        self.pwm0 = None
        self.pwm1 = None
        self.setup()

    def setup(self):
        self.pwm0 = PWMModule(pwm_channel=0)
        self.pwm1 = PWMModule(pwm_channel=1)

    def update(self, pwm_num, freq, duty):
        if pwm_num == 1:
            pwm_module = self.pwm1
        else:
            pwm_module = self.pwm0

        try:
            pwm_module.update(freq, duty)
        except HardwarePWMException as e:
            rospy.logerr("Wrong Hardware values were given!")
            rospy.logerr(str(e))


pwm_controller = PWMController()
#pwm_controller = PWMSoftware()


def callback(data):
    rospy.loginfo(data)
    for p in data.pwms:
        rospy.loginfo(rospy.get_caller_id() + f"Servo num: {p.pwm_num}, Freq = {p.freq}, DC = {p.duty}%")
        pwm_controller.update(p.pwm_num, p.freq, p.duty)

    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('pwm_pi_node', anonymous=True)

    rospy.Subscriber("pwm_listener", PWMArray, callback)

    rospy.loginfo("Started PWM Node")
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    

if __name__ == '__main__':
    
    listener()
