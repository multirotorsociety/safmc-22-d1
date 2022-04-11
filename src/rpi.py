#!/usr/bin/env python3
import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import Bool

class Drone:
    def __init__(self):
        '''
        ros subscriber
        '''
        self.gpio_sub = rospy.Subscriber('/gpio',Bool,self.gpio_callback)
        rospy.loginfo("Drone initialized!")
    # Callback functions
    def gpio_callback(self, msg):
        GPIO.output(23,msg.data)
def main():
    rospy.init_node('rpi_node', anonymous=True)
    drone = Drone()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
if __name__ == '__main__':
    main()