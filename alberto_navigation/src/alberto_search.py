#!/usr/bin/env python3

import math
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Twist, Vector3
from std_msgs.msg import Int16,Bool
from move_base_msgs.msg import MoveBaseActionResult
from sensor_msgs.msg import Imu

"""
entities present:
- coords_listener -> listens for current coordinates of the robot
- goal_listener -> listens for event indicating that the robot has reached its current goal
- goal_publisher -> publishes new robot goal

search flow:
(1) a new mission pops up: find object A
(2) coords_listener listens to find out current position
(3) search function figures out optimal trajectory depending on current position
(4) this returns an ordered list of locations
(5) goal_publisher fetches coordinates of first location and publishes them, triggering the robot to go
(6) goal_listener listens for goal reaching event
(7) when goal has been reached, turn 360º and check if the object is in sight
(8) if so, return
(9) if not, repeat steps (5)-(8) for next location on the list

possible robot states, in order: (coords_listener / goal_listener / goal_publisher)
- dormant: the search class has been initialized but there is no mission currently going on
    - off / on / off
- ready_for_path: based on the robot's current coordinates, we want to compute the optimal search path
    - off / off / off
    - here we call search function and save list of locations #!
- defining_path: computing the optimal search path
    - off / off / off
- defining_goal: we want to announce the next goal location
    - off / off / on
- travelling: we want to listen for when we have reached the goal location
    - off / on / off
- ready_for_turn: we want to turn 360º and look for object
    - off / off / off
    - call turn function -> this function changes the robot's state according to rules (8) and (9) #!
- turning: we are turning 360º and looking for object
    - off / off / off
"""


class Search():
    def __init__(self):
        rospy.init_node('alberto_search', anonymous=True)
        self.house_rooms = {
            # connections between rooms
            'connections' : {
                'bedroom1L' : {
                    'bedroom1L': 0,
                    'bedroom1R': 2.52
                },
                'bedroom1R': {
                    'bedroom1L': 2.52,
                    'bedroom1R': 0,
                    'hall1': 1.58
                },
                'bedroom2T' : {
                    'bedroom2T': 0,
                    'bedroom2B': 2.28
                },
                'bedroom2B' : {
                    'bedroom2B': 0,
                    'bedroom2T': 2.28,
                    'hall1': 2.51,
                    'hall2': 3.53
                },
                'bedroom3T': {
                    'bedroom3T': 0,
                    'bedroom3B': 1.70
                },
                'bedroom3B': {
                    'bedroom3B': 0,
                    'bedroom3T': 1.70,
                    'hall2': 2.16
                },
                'hall1': {
                    'hall1': 0,
                    'bedroom1R': 1.58,
                    'bedroom2B': 2.51,
                    'dining_room': 2.45
                },
                'hall2': {
                    'hall2': 0,
                    'bedroom2B': 3.53,
                    'bedroom3B': 2.16,
                    'toilet1': 1.40,
                    'vestibule': 2.20
                },
                'toilet1': {
                    'toilet1': 0,
                    'hall2': 1.40
                },
                'toilet2': {
                    'toilet2': 0,
                    'vestibule': 1.37
                },
                'vestibule': {
                    'vestibule': 0,
                    'hall2': 2.20,
                    'toilet2': 1.37,
                    'living_roomR': 3.10
                },
                'dining_room': {
                    'dining_room': 0,
                    'hall1': 2.45,
                    'kitchen': 2.59,
                    'living_roomL': 1.92,
                    'living_roomM': 2.07,
                    'terraceT': 3.03
                },
                'terraceT': {
                    'terraceT': 0,
                    'dining_room': 3.03,
                    'terraceB': 4.61
                },
                'terraceB': {
                    'terraceB': 0,
                    'terraceT': 4.61,
                    'living_roomL': 1.60
                },
                'living_roomL': {
                    'living_roomL': 0,
                    'terraceB': 1.60,
                    'living_roomM': 2.04,
                    'dining_room': 1.92
                },
                'living_roomM': {
                    'living_roomM': 0,
                    'living_roomL': 2.04,
                    'living_roomR': 2.45,
                    'dining_room': 2.07
                },
                'living_roomR': {
                    'living_roomR': 0,
                    'living_roomM': 2.45,
                    'vestibule': 3.10
                },
                'kitchen': {
                    'kitchen': 0,
                    'dining_room': 2.59
                }
            },
            # coordinates of each room
            'coordinates': {
                # R: (x, y)
                'bedroom1L': (-7.37, 3.09),
                'bedroom1R': (-4.87, 2.80),
                'bedroom2T': (-2.80, 4.96),
                'bedroom2B': (-3.42, 2.77),
                'bedroom3T': (-0.51, 4.48),
                'bedroom3B': (-0.59, 2.78),
                'hall1': (-5.49, 1.35),
                'hall2': (-0.62, 0.62),
                'vestibule': (0.29, -1.38),
                'kitchen': (-2.28, -0.88),
                'dining_room': (-4.87, -1.02),
                'toilet1': (0.76, 0.86),
                'toilet2': (1.66, -1.37),
                'living_roomL': (-5.92, -2.63),
                'living_roomM': (-3.89, -2.84),
                'living_roomR': (-1.64, -3.81),
                'terraceT': (-7.17, 0.96),
                'terraceB': (-7.15, -3.65)
            }
        }

        goal_cancel_topic = rospy.get_param("/goal_cancel_cmd",default="/goal_cancel")
        # self.searched_rooms = []
        self.search_path = None

        # rospy.init_node('search_pubsub', anonymous=True)
        self.current_coords = None
        self.current_orientation = None
        self.goal_reached   = False
        self.goal_object    = None
        self.object_found   = False
        self.final_stop     = False
        self.mission_active = False
        self.next_stop = None
        
        self.coords_listener    = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.coords_listener_callback)
        # self.goal_listener      = rospy.Subscriber("/move_base/status",GoalStatusArray,self.goal_listener_callback)
        self.goal_listener      = rospy.Subscriber("/move_base/result",MoveBaseActionResult,self.goal_listener_callback)
        self.mission_listener   = rospy.Subscriber("/active_mission_ID",Int16,self.mission_listener_callback)
        self.orientation_listener = rospy.Subscriber('/imu', Imu, self.orientation_listener_callback)
        self.goal_publisher     = rospy.Publisher('/goal_coords', Point, queue_size=10)
        self.turning_publisher  = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
        self.nav_goal_cancel_pub = rospy.Publisher(goal_cancel_topic,Bool,queue_size=10)

        self.state = 'dormant'
        self.run()

    def reset(self):
        self.search_path    = None
        # self.current_coords = None
        self.goal_reached   = False
        self.goal_object    = None
        self.object_found   = False
        self.final_stop     = False
        self.mission_active = False
        self.state          ='dormant'
        self.next_stop      = None
        self.object_found   = False

    def run(self):
        while not rospy.is_shutdown():
            if self.state != 'dormant':
                if self.state == 'travelling':
                    rospy.loginfo(str(self.state) + " to " + str(self.next_stop))
                else:
                    rospy.loginfo(self.state)
            #! Temp sleep
            rospy.sleep(1)
            if self.state == 'ready_for_path':
                self.find_optimal_search_path()
            elif self.state == 'ready_for_next_stop':
                rospy.loginfo("POPPED AN ELEMENT")
                next_stop = self.search_path.pop(0)
                self.next_stop = next_stop # just for printing
                next_stop_coords = self.house_rooms['coordinates'][next_stop]
                self.go(next_stop_coords)
            
            # elif self.state == 'defining_path':
            #     pass
            # elif self.state == 'travelling':
            #     pass
            
            elif self.state == 'ready_for_turn':
                self.turn()
                rospy.loginfo(self.search_path)

                if len(self.search_path) == 0:
                    self.final_stop = True
                    rospy.loginfo("Final stop true")

                if self.final_stop:
                    rospy.loginfo("self-finish is true")
                    # self.mission_active = False
                    # self.state = 'dormant'
                    # self.final_stop = False
                    self.reset()
            # What is happening is that its leavning turn withouth setting self.finish
                
            # elif self.state == 'turning':
            #     pass

    # def reset(self):
    #     self.searched_rooms = []

    # #! TODO
    # def find_room_by_coords(self, coords):

    #     pass

    def find_closest_room(self, coords, searched_rooms):
        x1, y1 = coords
        min_d, closest_room = float('inf'), None
        # current_room = self.find_room_by_coords(coords)

        
        current_room_d, current_room = float('inf'), None

        # See what room the robot is in
        for room, room_coords in self.house_rooms['coordinates'].items():
            d = math.sqrt((room_coords[0]-x1)**2 + (room_coords[1]-y1)**2)
            if d < current_room_d:
                current_room_d = d
                current_room = room

        # connections in which the current room is involved
        # connections = [conn for conn in self.house_rooms['connections'] if (conn[0]==current_room or conn[1]==current_room)]
        connections = self.house_rooms['connections'][current_room]
        for room, d in connections.items():
            # room = conn[0] if conn[1]==current_room else conn[1] # room we might potentially go to
            if room not in searched_rooms and d < min_d:
                #! this could be simplified by using the distances from the dict
                # x2, y2 = self.house_rooms['coordinates'][room]
                # d = math.sqrt((x2-x1)**2 + (y2-y1)**2)
                # d = self.house_rooms['connections']
                # if d < min_d:
                min_d = d
                closest_room = room

        #Problem : Whenever the robot is backed into a dead end, the only way to exit is through a already searched room
        #Solution : Whenever the robot can't find a non searched room, it should look into the connections of the next room
        
        #*Initially should search in adjacent to current room
        possible_adjacent_rooms = list(self.house_rooms['connections'][current_room].items())
        adjacent_rooms_tested = []

        while closest_room is None: # This should mean the robot is backed into a dead end
            # .items() returns a special not subscriptable dict_keys object, not a list
            
            #!In a 2nd order dead end,the robot cant chose the adjacent room which leads go doing deeper into the dead end
            min_adj_distance,adjacent_room = float('inf'),None
            for possible_room,distance in possible_adjacent_rooms:
                if possible_room not in adjacent_rooms_tested and distance<min_adj_distance:
                    min_adj_distance = distance
                    adjacent_room = possible_room
                    adjacent_rooms_tested.append(adjacent_room)

            # rospy.loginfo("Adjacent room is " + str(adjacent_room))

            if adjacent_room is None:
                break

            connections = self.house_rooms['connections'][adjacent_room]
            for room, d in connections.items():
                # room = conn[0] if conn[1]==current_room else conn[1] # room we might potentially go to
                if room not in searched_rooms and d < min_d:
                    min_d = d
                    closest_room = room
        
            #Updating room for next search
            possible_adjacent_rooms = list(self.house_rooms['connections'][adjacent_room].items())
        
        #! Is there anyway this while gets stuck?
        # The while is needed in case the robot is backed into a "2nd order dead end"

        return closest_room
    
    def go(self, coords): # publish to /goal_coords (geometry_msgs.msg Point)
        self.state = 'travelling'
        coords_msg = Point(x=coords[0], y=coords[1], z=0)
        self.goal_publisher.publish(coords_msg)

    # def goalReachedCallback(self):
    #     pass

    def turn(self):
        self.state = 'turning'
        while not self.current_orientation:
            pass

        turning = False
        init_orient = self.current_orientation[2] if (self.current_orientation[2]>0) else (2*math.pi + self.current_orientation[2]) # z value (0 to 2*pi)

        while True:
            linear = Vector3(x=0, y=0, z=0)
            angular = Vector3(x=0, y=0, z=1)
            new_turn = Twist(linear=linear, angular=angular)
            self.turning_publisher.publish(new_turn)

            current = self.current_orientation[2] if (self.current_orientation[2]>0) else (2*math.pi + self.current_orientation[2])

            if not turning and current != init_orient:
                turning = True
            if turning and current > init_orient: # if we have completed a full turn
                break

        rospy.loginfo(self.state)
        self.state = 'ready_for_next_stop'
        # TODO: update self.object_found according to results

    def find_optimal_search_path(self):
        if self.current_coords: # if the robot doesn't know its own position yet, this function will be continuously called until it does
            self.state = 'defining_path' # this way, find_optimal_search_path is only called once per mission
            path = []
            starting_coords = self.current_coords

            

            while len(path) < len(self.house_rooms['coordinates']):
                new_room = self.find_closest_room(starting_coords, path)

                #TODO! temporary, for some reason the kitchen isnt being considered
                if new_room is None:
                    break 

                path.append(new_room)
                rospy.loginfo("New room to add to path is " + str(new_room))
                # rospy.loginfo("Current path is " + str(path))
                new_room_coords = self.house_rooms['coordinates'][new_room]
                starting_coords = new_room_coords
            self.search_path = path
            rospy.loginfo("Final path is "+str(self.search_path))
            self.state = 'ready_for_next_stop'

            # start_room = self.find_closest_room(self.current_coords)
            # start_room_coords = self.house_rooms['coordinates'][start_room]
            # self.go(start_room_coords)
            # found = self.turn()
            # self.searched_rooms.append(start_room)

            # # while there are still rooms to search
            # while not found and len(self.searched_rooms) < len(self.house_rooms['coordinates']):
            #     next_room = self.find_closest_room(start_room_coords, self.searched_rooms)
            #     if next_room:
            #         next_room_coords = self.house_rooms['coordinates'][next_room]
            #         self.go(next_room_coords)
            #         found = self.turn()
            #         self.searched_rooms.append(next_room)
            #     else:
            #         break

        else:
            #! don't know if this works but the intent was to have a 2 second delay between each call to this function
            #! needs testing
            rospy.sleep(2) # rospy.sleep
 

    def coords_listener_callback(self, data):
        position = data.pose.pose.position
        self.current_coords = (position.x, position.y)
        # rospy.loginfo('CALLBACK CALLBACK CALLBACK')
        # rospy.loginfo(self.__str__())


    def goal_listener_callback(self,data):
        
        if not self.mission_active:
            return

        try:
            goal_status_int = data.status.status    #* 1 for active, 3 for completed with success
        except:
            self.goal_reached = False
            return
        
        if goal_status_int == 3:
            self.goal_reached = True
            
            # Should only transition to ready for turn if its traveling
            self.state = 'ready_for_turn' if self.state == 'travelling' else self.state 

            #! This cant be here as now its only updated in the end once
            # if len(self.search_path) == 0:
            #     self.final_stop = True


            #     self.state = 'ready_for_turn'
            # else:
            #     self.state = 'ready_for_next_stop'
        else:  
            self.goal_reached = False
            #TODO! THIS SHOULD MEAN MISSION ABORTED

    
    def cancelNavgoal(self):
        rospy.loginfo("CANCELING NAV GOAL")
        msg = Bool()
        msg.data = True

        self.nav_goal_cancel_pub.publish(msg)

    def mission_listener_callback(self, data):
        #! TODO: setup mission from data
        self.reset()
        self.cancelNavgoal()
        data = data.data
        
        if data == 20:
            self.mission_description = "search_pink_ball_in_house"
            self.goal_object = "single_pink_ball"
            self.mission_active = True
            self.state = 'ready_for_path'
            
        elif data == 23:
            self.mission_description = "check_if_someone_is_home"
            self.goal_object = "single_person"
            self.mission_active = True
            self.state = 'ready_for_path'
            
        elif data == 24:
            self.mission_description = "count_num_of_cubes_in_house"
            self.goal_object = "count_blue_cubes"
            self.mission_active = True
            self.state = 'ready_for_path'

    def orientation_listener_callback(self, data):
        quaternion_orientation = data.orientation
        self.current_orientation = self.euler_from_quaternion(
            quaternion_orientation.x,
            quaternion_orientation.y,
            quaternion_orientation.z,
            quaternion_orientation.w)

    def euler_from_quaternion(self, x, y, z, w):
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
            

    # def init_publisher(self):
    #     rospy.loginfo('INIT PUBLISHER !!!!!!!!!!!!!!!!!!!!!!!!!!!!')
    #     pub = rospy.Publisher('/goal_coords', Point, queue_size=10)
    #     return pub

    def __str__(self):
        return "robot position: " + str(self.current_coords)

search = Search()