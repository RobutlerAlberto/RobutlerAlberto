#!/usr/bin/env python3

import math
import rospy
from geometry_msgs.msg import Twist, Quaternion, Vector3
from sensor_msgs.msg import Imu
 
current_orientation = None

def euler_from_quaternion(x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

def orientation_listener_callback(data):
    global current_orientation
    quaternion_orientation = data.orientation
    current_orientation = euler_from_quaternion(
        quaternion_orientation.x,
        quaternion_orientation.y,
        quaternion_orientation.z,
        quaternion_orientation.w)

def main():
    rospy.loginfo("STARTING TO TURN SOON")
    rospy.Subscriber('/imu', Imu, orientation_listener_callback)
    rospy.sleep(3)
    turning_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
    
    turning = False
    init_orient = current_orientation[2] # z value
    while not rospy.is_shutdown():
        linear = Vector3(x=0, y=0, z=0)
        angular = Vector3(x=0, y=0, z=10)
        new_turn = Twist(linear=linear, angular=angular)
        turning_publisher.publish(new_turn)

        if not turning and current_orientation[2] != init_orient:
            rospy.loginfo('TURNING!')
            turning = True

        if turning and current_orientation[2] == init_orient: # if we have completed a full turn
            rospy.loginfo('COMPLETED A FULL TURN!')
            break
        
main()