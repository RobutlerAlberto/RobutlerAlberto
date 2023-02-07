#!/usr/bin/env python3

import rospy

from functools import partial

from std_msgs.msg import String,Header,Int16,Bool
from geometry_msgs.msg import Point
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseActionResult

class Mission():
    #Class variable
    mission_ids = {
        "free"              : 0,
        
        # Go to
        "go_to_bedroom1"    : 1,
        "go_to_bedroom2"    : 2,
        "go_to_bedroom3"    : 3,
        "go_to_hall"        : 4,
        "go_to_vestibule"   : 5,
        "go_to_kitchen"     : 6,
        "go_to_dining_room" : 7,
        "go_to_toilet1"     : 8,
        "go_to_toilet2"     : 9,
        "go_to_living_room" : 10,
        "go_to_terrace"     : 11,
        # Passive
        "search_pink_ball_in_house"     : 20,
        "check_if_table_free"           : 21,
        "take_photograph"               : 22,
        "check_if_someone_is_home"      : 23,
        "count_num_of_cubes_in_house"   : 24,
        
        #Active
        "touch_person"      : 30
        }

    goal_cancel_topic = rospy.get_param("/goal_cancel_cmd",default="/goal_cancel")

    def __init__(self):
        self.ID = 0
        self.global_status = False 
        self.nav_goal_aborted = False
        self.nav_goal_reached = False
        self.cancel_order = False

        #*ROS entities
        self.mission_id_sub  = rospy.Subscriber("/active_mission_ID",Int16,self.missionIdSubscriberCallback)
        # self.nav_goal_status_sub = rospy.Subscriber("/move_base/status",GoalStatusArray,self.nav_goal_status_callback)
        self.nav_goal_result_sub = rospy.Subscriber("/move_base/result",MoveBaseActionResult,self.nav_goal_result_callback)
        
        self.nav_goal_cancel_pub = rospy.Publisher(Mission.goal_cancel_topic,Bool,queue_size=10)

    def reset(self): 
        self.ID = 0
        self.global_status = False 
        self.nav_goal_aborted = False
        self.nav_goal_reached = False
        self.cancel_order = False

    def missionIdSubscriberCallback(self,msg):
        self.ID = msg.data

        if self.ID == 0:
            self.cancelNavgoal()
            self.reset()        


        # if self.ID in GoTo.go_to_id_coords.keys():
        #     self.cancelNavgoal()
        #     self.global_status = True
        #     rospy.loginfo("Mission  active")

        
        # rospy.loginfo("mission id is" + str(self.ID))

    def cancelNavgoal(self):
        rospy.loginfo("CANCELING NAV GOAL")
        msg = Bool()
        msg.data = True

        self.nav_goal_cancel_pub.publish(msg)
    
    def nav_goal_status_callback(self,data): #! Only use if constant updates on goal are needed

        #If its none, shouldn't do nothing        
        if not self.global_status:
            return
        
        # Check if it exists
        try:
            # nav_goal_status_int = data.status_list[0].status    # 3 for completed with success
            nav_goal_status_int = data.status_list[0].status    #* 1 for active, 3 for completed with success
        except:
            self.nav_goal_reached = False
            return

        
        #Assign nav goal status
        if nav_goal_status_int == 3:    #Done with success
            self.nav_goal_reached = True

        elif nav_goal_status_int == 1 : #In progress
            self.nav_goal_reached = False

        else:                           #Something failed, should abort
            self.nav_goal_aborted = True
        rospy.loginfo("Nav goal reached is "+ str(self.nav_goal_reached))


    def nav_goal_result_callback(self,data):

        #If its none, shouldn't do nothing        
        if not self.global_status:
            return
        
        # Check if it exists
        try:
            nav_goal_status_int = data.status.status    #* 3 for completed with success
        except:
            self.nav_goal_reached = False
            return
        
        # rospy.loginfo(nav_goal_status_int)
        #Assign nav goal status
        if nav_goal_status_int == 3:    #Done with success
            self.nav_goal_reached = True
        else:                           #Something failed, should abort
            self.nav_goal_aborted = True
        # rospy.loginfo("Nav goal reached is "+ str(self.nav_goal_reached))
