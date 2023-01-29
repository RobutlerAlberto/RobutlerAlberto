#!/usr/bin/env python3

import rospy
import sys

from functools import partial

from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovariance,Pose,PoseWithCovarianceStamped

def main():
    #-----------------------------
    # Initialization
    #-----------------------------
    rospy.init_node("alberto_initial_pose_setter",anonymous=False)
    rate  = rospy.Rate(10) 
    x_pos = rospy.get_param("~x_pose",-1.5)
    y_pos = rospy.get_param("~y_pose",-4)
    z_pos = rospy.get_param("~z_pose",0)

    pose_pub = rospy.Publisher("/initialpose",PoseWithCovarianceStamped,queue_size=10)

    initial_pose = PoseWithCovarianceStamped()

    initial_pose.header.frame_id = "map"
    initial_pose.header.stamp = rospy.Time.now()

    initial_pose.pose.pose.position.x = x_pos
    initial_pose.pose.pose.position.x = x_pos
    initial_pose.pose.pose.position.y = y_pos

    initial_pose.pose.pose.orientation.x = 0
    initial_pose.pose.pose.orientation.y = 0
    initial_pose.pose.pose.orientation.z = 0
    initial_pose.pose.pose.orientation.w = 1

    initial_pose.pose.covariance = [0]*36 # No uncertainty in position
    #-----------------------------
    # Processing
    #-----------------------------

    # while not rospy.is_shutdown() :
    #     pose_pub.publish(initial_pose)


    #! It should only publish after the first subscriber connects to this node
    while(pose_pub.get_num_connections() == 0 and not rospy.is_shutdown()):
        rate.sleep()

    # In case something else happens to connect to initial pose
    rospy.sleep(1)

    pose_pub.publish(initial_pose)
    #-----------------------------
    # Termination
    #-----------------------------
    

    rospy.signal_shutdown(shutdownCallback) # Shutdowns node


def shutdownCallback():
    rospy.loginfo("Initial pose published, shutting down node")
    sys.exit() # Stops script from running, killing node


if __name__=="__main__":
    main()
