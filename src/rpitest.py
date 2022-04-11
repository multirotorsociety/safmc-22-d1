#!/usr/bin/env python3
from json.decoder import JSONDecodeError
import guli
import rospy
from std_msgs.msg import Bool

class Drone:
    def __init__(self):
        '''
        ros subscriber
        '''
        self.gpio_sub = rospy.Publisher('/gpio',Bool, queue_size=10)
        rospy.loginfo("Drone initialized!")
    def start(self):
        while True:
            try:
                rate = rospy.Rate(10)
                em = bool(guli.GuliVariable("EM").get())
                self.gpio_sub.publish(em)
                rate.sleep()
            except KeyboardInterrupt:
                return
            except JSONDecodeError:
                print('json error, trying again')
                continue
    # Callback functions


def main():
    guli.GuliVariable("EM").setValue(1)
    rospy.init_node('rpi_node', anonymous=True)
    drone = Drone()
    drone.start()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
if __name__ == '__main__':
    main()