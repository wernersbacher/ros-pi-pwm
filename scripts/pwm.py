import logging
import time
import RPi.GPIO as GPIO

PWM1PIN = 12  # adapt to your wiring
PWM2PIN = 33  # adapt to your wiring
fPWM = 50  # Hz (not higher with software PWM)



class PWM:

    def __init__(self):
        self.pwm1 = None
        self.pwm2 = None
        self.setup()


    def setup(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(PWM1PIN, GPIO.OUT)
        GPIO.setup(PWM2PIN, GPIO.OUT)
        self.pwm1 = GPIO.PWM(PWM1PIN, fPWM)
        self.pwm2 = GPIO.PWM(PWM2PIN, fPWM)
        self.pwm1.start(0)
        self.pwm2.start(0)

    def set_dc(self, duty):
        self.pwm_engine.ChangeDutyCycle(duty)
        


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('pwm_pi_node', anonymous=True)

    rospy.Subscriber("pwm1_listener", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    

if __name__ == '__main__':
    listener()