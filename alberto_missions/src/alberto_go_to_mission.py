#!/usr/bin/env python3

import rospy

from functools import partial

from std_msgs.msg import String,Header,Int16,Bool
from geometry_msgs.msg import Point
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseActionResult
from alberto_mission import Mission

class GoTo(Mission):

    go_to_id_coords = {
                # R: (x, y)
                        1 : (-4.87, 2.80)   ,   #Bedroom 1
                        2 : (-3.42, 2.77)   ,   #Bedroom 2
                        3 : (-0.59, 2.78)   ,   #Bedroom 3
                        4 : (-5.49, 1.35)   ,   #Hall
                        5 : (0.29, -1.38)   ,   #Vestibule
                        6 : (-2.28, -0.88)  ,   #Kitchen
                        7 : (-4.87, -1.02)  ,   #Dining room
                        8 : (0.76, 0.86)    ,   #Toilet 1     
                        9 : (1.66, -1.37)   ,   #Toilet 2
                        10: (-1.64, -3.81)  ,   #Living room 
                        11: (-7.15, -3.65)      #Terrace
            }
    
    def __init__(self):
        super().__init__()
        self.goal_coords_pub = rospy.Publisher("/goal_coords",Point,queue_size=10)
        self.status = 'dormant'

    def reset(self): 
        super().reset()

        self.status = 'dormant'

    # Method differs from parent class
    def missionIdSubscriberCallback(self,msg):
        # When receiving a new ID, cancels current one and resets goal
        self.cancelNavgoal()
        self.reset()


        self.ID = msg.data
        if self.ID == 0:
            self.cancelNavgoal()
            self.reset()        
            
        if self.ID in GoTo.go_to_id_coords.keys():
            self.global_status = True
            rospy.loginfo("Mission  active")

        
        # rospy.loginfo("mission id is" + str(self.ID))

    def run(self):
        '''
        Flow of the Go To Missions
            <dormant>
        (1) Figure out which coordinates to send    
        (2) Send coordinates
            <travelling>
        (3) Wait for goal reached
            (3.1) If goal isn't reached after a delay, cancel the mission
                    (3.1.1) To cancel a mission the 2DNavGoal should be aborted
        '''
        if self.global_status == False:
            # rospy.loginfo("Mission not active")
            return

        rospy.loginfo("Entered run")
        # rospy.loginfo("mission id is" + str(self.ID))
        # rospy.loginfo("Key in dict of ids is " + str(self.ID in GoTo.go_to_id_coords.keys()))
        # Should only do mission if there isn't another mission active



        if self.status == 'dormant' and self.global_status == True: 
            
            self.goal_coords = GoTo.go_to_id_coords.get(self.ID) # get() returns none if the key is not found, dict['key'] yields error
            coords_msg = Point(x=self.goal_coords[0],y =self.goal_coords[1])
            self.goal_coords_pub.publish(coords_msg)
            self.status = 'travelling'

        if self.nav_goal_reached == True and self.status == 'travelling':
            self.reset() # Ends mission

    def cancelMission(self):
        self.cancelNavgoal()


def main():
    rospy.init_node("alberto_go_to_mission",anonymous=False)
    mission = GoTo()

    while not rospy.is_shutdown():
        
        mission.run()
        rospy.sleep(1)
        rospy.loginfo(mission.status)
        
        #! If this if is below, it would evaluate false if the global status was false and and goal aborted were true
        if mission.nav_goal_aborted == True:
            mission.cancelMission() #Cancels navgoal
            mission = GoTo() # Doing this sets everything back to zero and stops the mission

        rospy.loginfo("Global mission status is " + str(mission.global_status))
        # if mission.global_status == False:
        #     mission = GoTo() # If status is false, resets the class for next mission



if __name__=="__main__":
    main()

'''
- What happens if another go mission is called?
- What happens if another mission is called?
- What happens when a goal is aborted?
'''