#!/usr/bin/python3

import rospy
#from std_msgs.msg import String
from ros_pi_pwm.msg import command
from rpi_hardware_pwm import HardwarePWM

fPWM = 50  # Hz (not higher with software PWM)


class PWM:

    def __init__(self):
        self.pwm0 = None
        self.pwm1 = None
        self.setup()

    def setup(self):
        self.pwm0 = HardwarePWM(pwm_channel=0, hz=fPWM)
        self.pwm1 = HardwarePWM(pwm_channel=1, hz=fPWM)
        self.pwm0.start(0)
        self.pwm1.start(0)

    def set_dc(self, pwm_num, duty):
        """ todo: check types """
        
        if pwm_num == 1:
            self.pwm1.change_duty_cycle(duty)
        else:
            self.pwm0.change_duty_cycle(duty)
                   

pwm = PWM()

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + f"Servo num: {data.pwm_num}, DC = {data.duty}%")
    pwm.set_dc(data.pwm_num, data.duty)
    

    
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