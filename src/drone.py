#!/usr/bin/env python3
from json.decoder import JSONDecodeError
import rospy
# import math
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped, TwistStamped
from pyquaternion import Quaternion
from std_msgs.msg import Bool
import time, guli



class Drone:

    def __init__(self):
        self.local_pose = None
        self.arm_state = False
        self.offboard_state = False
        self.frame = "BODY"

        self.state = State()

        self.target_msg = PositionTarget()
        self.vel_msg = TwistStamped()
        self.mode = 0  # 0 for pose, 1 for velocity

        '''
        ros subscribers
        '''
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        self.mavros_sub = rospy.Subscriber("/mavros/state", State, self.mavros_state_callback)
        self.target_sub = rospy.Subscriber("/target", PositionTarget, self.target_callback)
        self.vel_sub = rospy.Subscriber("/cmd_vel", TwistStamped, self.vel_callback)

        '''
        ros publishers
        '''
        self.local_target_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self.gpio_pub = rospy.Publisher('/gpio', Bool,queue_size=10)
        
        '''
        ros services
        '''
        self.arm_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.flight_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        rospy.loginfo("Drone initialized!")

    def start(self):
        for i in range(10):
            if self.local_pose is not None:
                break
            else:
                print("Waiting for initialization.")
                time.sleep(0.5)

        print('up')
        self.target_msg = self.construct_vel_target(0, 0, 1)
        for i in range(10):
            self.local_target_pub.publish(self.target_msg)
            self.arm_state = self.arm()
            self.offboard_state = self.offboard()
            time.sleep(0.2)

        
        print('done up')
        for i in range(5):
            self.target_msg = self.construct_vel_target(0, 0, 0)
            self.local_target_pub.publish(self.target_msg)
            time.sleep(0.2)

        '''
        main ROS thread
        '''
        print("keybord control")
        guli.GuliVariable("p").setValue(0.0)
        guli.GuliVariable("r").setValue(0.0)
        guli.GuliVariable("a").setValue(0.0)
        guli.GuliVariable("y").setValue(0.0)
        while self.arm_state and self.offboard_state and not rospy.is_shutdown():
            try:
                rate = rospy.Rate(10)
                pitch = float(guli.GuliVariable("p").get())
                roll = float(guli.GuliVariable("r").get())
                accend = float(guli.GuliVariable("a").get()*0.4)
                yaw = float(guli.GuliVariable("y").get())
                self.target_msg = self.construct_vel_target(pitch, roll, accend, yaw)
                self.local_target_pub.publish(self.target_msg)
                rate.sleep()
            except KeyboardInterrupt:
                return
            except JSONDecodeError:
                print('json error, trying again')
                continue

    # Callback functions
    def mavros_state_callback(self, msg):
        ''' Debugging logging'''
        if self.state.armed != msg.armed:
            rospy.loginfo("armed state changed from {0} to {1}".format(
                self.state.armed, msg.armed))

        if self.state.connected != msg.connected:
            rospy.loginfo("connected changed from {0} to {1}".format(
                self.state.connected, msg.connected))

        if self.state.mode != msg.mode:
            rospy.loginfo("mode changed from {0} to {1}".format(
                self.state.mode, msg.mode))
        
        self.state = msg

    def local_pose_callback(self, msg):
        self.local_pose = msg

    def target_callback(self, msg):
        self.target_msg = msg
        self.mode = 0

    def vel_callback(self, msg):
        self.vel_msg = msg
        self.mode = 1

    def land_callback(self, msg):
        if msg.data == "LAND":
            self.land()
    def drop_callback(self,msg):
        pass

    # Helper functions
    def q2yaw(self, q):
        if isinstance(q, Quaternion):
            rotate_z_rad = q.yaw_pitch_roll[0]
        else:
            q_ = Quaternion(q.w, q.x, q.y, q.z)
            rotate_z_rad = q_.yaw_pitch_roll[0]

        return rotate_z_rad

    def arm(self):
        if self.arm_srv(True):
            return True
        else:
            print("Vehicle arming failed!")
            return False

    def disarm(self):
        if self.arm_srv(False):
            return True
        else:
            print("Vehicle disarming failed!")
            return False

    def offboard(self):
        if self.flight_mode_srv(custom_mode='OFFBOARD'):
            return True
        else:
            print("Vechile Offboard failed")
            return False

    def land(self):
        print("land called!")
        if self.flight_mode_srv(custom_mode='AUTO.LAND'):
            return True
        else:
            print("Vechile Land failed")
            return False
    

    def construct_vel_target(self,x=0.0, y=0.0, z=0.0,yr=0.0):
        target_raw_pose = PositionTarget()
        target_raw_pose.header.stamp = rospy.Time.now()

        target_raw_pose.coordinate_frame = 8

        target_raw_pose.velocity.x = x
        target_raw_pose.velocity.y = y
        target_raw_pose.velocity.z = z
        target_raw_pose.yaw_rate = yr 

        target_raw_pose.type_mask = 1479

        return target_raw_pose

def main():
    rospy.init_node('offboard_node', anonymous=True)
    drone = Drone()
    drone.start()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()
