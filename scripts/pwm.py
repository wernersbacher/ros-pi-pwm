#!/usr/bin/python3

import os
import os.path

import rospy
import math
#from std_msgs.msg import String
from ros_pi_pwm.msg import PWMArray, PWM
from rpi_hardware_pwm import HardwarePWM, HardwarePWMException


import RPi.GPIO as GPIO

P_SERVO = 12  # adapt to your wiring
P_ENGINE = 33  # adapt to your wiring
fPWM = 50


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
