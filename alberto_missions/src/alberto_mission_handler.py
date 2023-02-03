#!/usr/bin/env python3
import rospy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *

server = None

menu_handler = MenuHandler()

h_first_entry = 0
h_mode_last = 0

house_rooms = {
            # coordinates of each room
                # R: (x, y)
                # 'bedroom1L': (-7.37, 3.09),
                'Bedroom1': (-4.87, 2.80),
                # 'bedroom2T': (-2.80, 4.96),
                'Bedroom2': (-3.42, 2.77),
                # 'bedroom3T': (-0.51, 4.48),
                'Bedroom3': (-0.59, 2.78),
                'Hall': (-5.49, 1.35),
                # 'hall2': (-0.62, 0.62),
                'Vestibule': (0.29, -1.38),
                'Kitchen': (-2.28, -0.88),
                'Dining_room': (-4.87, -1.02),
                'Toilet1': (0.76, 0.86),
                'Toilet2': (1.66, -1.37),
                # 'living_roomL': (-5.92, -2.63),
                'Living_roomM': (-3.89, -2.84),
                # 'living_roomR': (-1.64, -3.81),
                # 'terraceT': (-7.17, 0.96),
                'Terrace': (-7.15, -3.65)
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

def goToMission(feedback):
    rospy.loginfo("The go to mission sub-menu has been found.")

def searchPinkBallMission(feedback):
    pass

def checkKitchenTableMission(feedback):
    pass

def takePhotoMission(feedback):
    pass

def searchPeopleMission(feedback):
    pass

def countNumOfCubesMission(feedback):
    pass


def touchPersonMission(feedback):
    pass

def initMenu():
    global house_rooms
    
    #*Passive mission
    passive_entry = menu_handler.insert( "Passive missions")

    menu_handler.insert("Search pink ball in the house",parent=passive_entry,callback=searchPinkBallMission)
    menu_handler.insert("Check if the kitchen's table is free",parent=passive_entry,callback=searchPinkBallMission)
    menu_handler.insert("Take a photograph",parent=passive_entry,callback=takePhotoMission)
    menu_handler.insert("Check if someone is home",parent=passive_entry,callback=searchPeopleMission)
    menu_handler.insert("Check if someone is home",parent=passive_entry,callback=searchPeopleMission)
    menu_handler.insert("Count number of blue cubes in the house",parent=passive_entry,callback=countNumOfCubesMission)


    #*Active mission
    active_entry = menu_handler.insert( "Active missions")

    menu_handler.insert("Touch people",parent=active_entry,callback=touchPersonMission)
    #* Go to missions
    sub_menu_handle = menu_handler.insert( "Go to" )

    for division in house_rooms.keys():
        menu_handler.insert( str(division), parent=sub_menu_handle, callback=goToMission );

if __name__=="__main__":
    rospy.init_node("menu")
    
    server = InteractiveMarkerServer("menu")

    initMenu()
    
    makeMenuMarker( "marker1" )

    menu_handler.apply( server, "marker1" )
    server.applyChanges()

    rospy.spin()

    # while not rospy.is_shutdown:
    #     pass



