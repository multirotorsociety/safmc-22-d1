#!/usr/bin/env python3
from gpiozero import LED
import rospy
from std_msgs.msg import Bool
x=LED(16)
class Drone:
    def __init__(self):
        '''
        ros subscriber
        '''
        self.gpio_sub = rospy.Subscriber('/gpio',Bool,self.gpio_callback)
        rospy.loginfo("Drone initialized!")
    # Callback functions
    def gpio_callback(self, msg):
        x.value = msg.data


def main():
    rospy.init_node('rpi_node', anonymous=True)
    drone = Drone()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
if __name__ == '__main__':
    main()
