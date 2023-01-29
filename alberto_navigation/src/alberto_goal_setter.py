#!/usr/bin/env python3

import rospy

from functools import partial

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped

def main():
    #-----------------------------
    # Initialization
    #-----------------------------
    rospy.init_node("alberto_goal_setter",anonymous=False)

    goal_pub = rospy.Publisher("/move_base_simple/goal",PoseStamped,queue_size = 10)
    


    #-----------------------------
    # Processing
    #-----------------------------



    #-----------------------------
    # Termination
    #-----------------------------

if __name__=="__main__":
    main()
