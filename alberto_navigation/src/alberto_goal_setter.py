#!/usr/bin/env python3

import rospy

from functools import partial

from std_msgs.msg import Header,Bool
from geometry_msgs.msg import PoseStamped,Point 
from actionlib_msgs.msg import GoalID


def main():
    #-----------------------------
    # Initialization
    #-----------------------------
    rospy.init_node("alberto_goal_setter",anonymous=False)
    goal_coords_topic = rospy.param = rospy.get_param("/goal_coords_topic", default="/goal_coords")
    goal_cancel_topic = rospy.get_param("/goal_cancel_cmd",default="/goal_cancel")
    

    kwargs = {}
    kwargs["goal_pub"] = rospy.Publisher("/move_base_simple/goal",PoseStamped,queue_size = 10)
    kwargs["cancel_pub"] = rospy.Publisher("/move_base/cancel",GoalID,queue_size = 10)

    # Defining partials
    coordsReceivedCallbackPart = partial(coordsReceivedCallback,**kwargs)
    cancelGoalCallbackPart = partial(cancelGoalCallback,**kwargs)

    goal_coords_sub = rospy.Subscriber(goal_coords_topic,Point,coordsReceivedCallbackPart) 
    goal_cancel_sub = rospy.Subscriber(goal_cancel_topic,Bool,cancelGoalCallbackPart) 



    rate = rospy.Rate(10)


    #-----------------------------
    # Processing
    #-----------------------------
    rospy.spin()

    

def coordsReceivedCallback(message, **kwargs):
    #* Message received should be of type geometry_msgs/Point


    # Definining goal message
    goal_message = PoseStamped()

    goal_message.header.frame_id = "map"
    goal_message.header.stamp = rospy.Time.now()
    
    #! Kitchen coords 
    # goal_message.pose.position.x = -2.28
    # goal_message.pose.position.y = -0.88
    
    goal_message.pose.position.x = message.x
    goal_message.pose.position.y = message.y
    goal_message.pose.position.z = 0 #* Should be 0 for 2D NavGoal

    #TODO  Final orientation will always be the same, which could be better
    goal_message.pose.orientation.x = 0
    goal_message.pose.orientation.y = 0
    goal_message.pose.orientation.z = 0
    goal_message.pose.orientation.w = 1


    # Publishing goal message
    kwargs["goal_pub"].publish(goal_message)

def cancelGoalCallback(message,**kwargs):

    # If cancel is false return
    if not message.data:
        return 

    cancel_msg = GoalID()
    cancel_msg.id = ''
    kwargs['cancel_pub'].publish(cancel_msg)



if __name__=="__main__":
    main()
