#!/usr/bin/env python3
import rospy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *

from std_msgs.msg import Int16

server = None

menu_handler = MenuHandler()

h_first_entry = 0
h_mode_last = 0

house_rooms = {
            # coordinates of each room
                # R: (x, y)
                # 'bedroom1L': (-7.37, 3.09),
                'Bedroom1'  : (-4.87, 2.80),
                # 'bedroom2T': (-2.80, 4.96),
                'Bedroom2'  : (-3.42, 2.77),
                # 'bedroom3T': (-0.51, 4.48),
                'Bedroom3'  : (-0.59, 2.78),
                'Hall'      : (-5.49, 1.35),
                # 'hall2': (-0.62, 0.62),
                'Vestibule' : (0.29, -1.38),
                'Kitchen'   : (-2.28, -0.88),
                'Dining_room': (-4.87, -1.02),
                'Toilet1'   : (0.76, 0.86),
                'Toilet2'   : (1.66, -1.37),
                # 'living_roomL': (-5.92, -2.63),
                'Living_roomM': (-3.89, -2.84),
                # 'living_roomR': (-1.64, -3.81),
                # 'terraceT': (-7.17, 0.96),
                'Terrace'   : (-7.15, -3.65)
            }


def makeBox( msg ):
    marker = Marker()

    marker.type = Marker.CUBE
    marker.scale.x = msg.scale * 0.45
    marker.scale.y = msg.scale * 0.45
    marker.scale.z = msg.scale * 0.45
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 0.0

    return marker

def makeBoxControl( msg ):
    control = InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append( makeBox(msg) )
    msg.controls.append( control )
    return control

def makeEmptyMarker( dummyBox=True ):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_footprint"
    
    int_marker.pose.position.x = 0.0 
    int_marker.pose.position.y = 0.0 
    int_marker.scale = 1
    return int_marker

def makeMenuMarker( name ):
    int_marker = makeEmptyMarker()
    int_marker.name = name

    control = InteractiveMarkerControl()

    control.interaction_mode = InteractiveMarkerControl.BUTTON
    control.always_visible = True

    control.markers.append( makeBox( int_marker ) )
    int_marker.controls.append(control)

    server.insert( int_marker )



class MissionHandler():

    def __init__(self):
        self.mission_ids = [
        ("free"              ,0),
        
        # Go to
        ("go_to_bedroom1"    , 1),
        ("go_to_bedroom2"    , 2),
        ("go_to_bedroom3"    , 3),
        ("go_to_hall"        , 4),
        ("go_to_vestibule"   , 5),
        ("go_to_kitchen"     , 6),
        ("go_to_dining_room" , 7),
        ("go_to_toilet1"     , 8),
        ("go_to_toilet2"     , 9),
        ("go_to_living_room" , 10),
        ("go_to_terrace"     , 11),
        # Passive
        ("search_pink_ball_in_house"          , 20),
        ("check_if_table_free"                , 21),
        ("take_photograph"                    , 22),
        ("check_if_someone_is_home"           , 23),
        ("count_num_of_blue_cubes_in_house"   , 24),
        ("search_red_ball_in_house"           , 25), #! Por implementar
        ("search_computer"                    , 26), #! Por implementar
        ("see_if_table_is_empty"              , 27), #! Por implementar
        ("spawn_object"                       , 28), #! Por implementar
        #Active
        ("touch_person"      , 30)
        ]

        self.current_mission_id = 0
        self.active_mission_id_pub = self.initMissionIDPub()

    def initMissionIDPub(self):
        pub = rospy.Publisher("/active_mission_ID",Int16,queue_size=10)
        return pub

    def goToMission(self,feedback): 
        '''
        Not very happy with this, as there is nothing defining the entry other than ID, having to identify them by ID
        makes it so if we change the order of the entries it breaks the function.
        The only solution im seeing would be to have a unique callback for each go to, which seems like a ton of repeated code
        '''
        entry_id = feedback.menu_entry_id
        # rospy.loginfo(entry_id)

        #!Room's ID are from 1 to 12
        mission_division = self.mission_ids[entry_id-1][0]
        mission_id = self.mission_ids[entry_id-1][1]
        rospy.loginfo("Ordering to start mission " + str(mission_division) + " (ID "+str(mission_id)+")")

        self.active_mission_id_pub.publish(mission_id)

    def genericMission(self,mission_description):
        for i in range(len(self.mission_ids)):
            if self.mission_ids[i][0] == mission_description:
                mission_description = self.mission_ids[i][0]
                mission_id = self.mission_ids[i][1]

        rospy.loginfo("Ordering to start mission " + str(mission_description) + " (ID "+str(mission_id)+")")
        self.active_mission_id_pub.publish(mission_id)

    def searchPinkBall(self,feedback):
       self.genericMission("search_pink_ball_in_house") 

    def checkKitchenTable(self,feedback):
       self.genericMission("check_if_table_free") 

    def takePhoto(self,feedback):
       self.genericMission("take_photograph") 

    def searchPeople(self,feedback):
       self.genericMission("check_if_someone_is_home") 
    
    def countNumOfCubes(self,feedback):
       self.genericMission("count_num_of_cubes_in_house") 

    def touchPerson(self,feedback):
       self.genericMission("touch_person") 

# End of MissionHandler() class 

def initMenu():
    global house_rooms,mission
    
    #* Go to missions
    #! This must be the first entries, so that i can know which division is clicked

    sub_menu_handle = menu_handler.insert( "Go to" ) # ID 1

    for division in house_rooms.keys(): # ID 2 - 12
        menu_handler.insert( str(division) ,parent=sub_menu_handle, callback=mission.goToMission );

    #*passive mission
    passive_entry = menu_handler.insert( "Passive missions")

    menu_handler.insert("search pink ball in the house"          ,parent=passive_entry  ,callback=mission.searchPinkBall)
    menu_handler.insert("check if the kitchen's table is free"   ,parent=passive_entry  ,callback=mission.checkKitchenTable)
    menu_handler.insert("take a photograph"                      ,parent=passive_entry  ,callback=mission.takePhoto)
    menu_handler.insert("check if someone is home"               ,parent=passive_entry  ,callback=mission.searchPeople)
    menu_handler.insert("count number of blue cubes in the house",parent=passive_entry  ,callback=mission.countNumOfCubes)


    #*active mission
    active_entry = menu_handler.insert( "Active missions")

    menu_handler.insert("Touch people",parent=active_entry,callback=mission.touchPerson)


if __name__=="__main__":
    rospy.init_node("menu")
    
    server = InteractiveMarkerServer("menu")

    mission = MissionHandler()
    initMenu()
    
    makeMenuMarker( "marker1" )

    menu_handler.apply( server, "marker1" )
    server.applyChanges()

    rospy.spin()

    # while not rospy.is_shutdown:
    #     pass



