#!/usr/bin/python3

import rospy
import math
#from std_msgs.msg import String
from ros_pi_pwm.msg import command
from rpi_hardware_pwm import HardwarePWM


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

        # create pwm module if not started
        if self.stopped:
            self.start()
            self.stopped = False
        else:
            self.pwm.change_duty_cycle(duty)
            if not is_new_freq:
                self.pwm.change_frequency(freq)

    def stop(self):
        self.stopped = True
        self.pwm = None


class PWM:
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

        pwm_module.update(freq, duty)
        

pwm = PWM()


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + f"Servo num: {data.pwm_num}, Freq = {data.freq}, DC = {data.duty}%")
    pwm.update(data.pwm_num, data.freq, data.duty)

    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('pwm_pi_node', anonymous=True)

    rospy.Subscriber("pwm_listener", command, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    

if __name__ == '__main__':
    
    listener()
