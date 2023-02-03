#!/usr/bin/env python3

import math
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped, Point32
from time import sleep


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
        self.house_rooms = {
            # connections between rooms
            'connections': [
                # (R1, R2, distance)
                ('bedroom1L', 'bedroom1R', 2.52),
                ('bedroom1R', 'hall1', 1.58),
                ('bedroom2T', 'bedroom2B', 2.28),
                ('bedroom2B', 'hall1', 2.51), 
                ('bedroom2B', 'hall2', 3.53), 
                ('bedroom3T', 'bedroom3B', 1.70), 
                ('bedroom3B', 'hall2', 2.16), 
                ('toilet1', 'hall2', 1.40), 
                ('vestibule', 'hall2', 2.20), 
                ('toilet2', 'vestibule', 1.37), 
                ('dining_room', 'hall1', 2.45), 
                ('dining_room', 'kitchen', 2.59), 
                ('dining_room', 'terraceT', 3.03), 
                ('terraceT', 'terraceB', 4.61), 
                ('terraceB', 'living_roomL', 1.60), 
                ('living_roomL', 'living_roomM', 2.04), 
                ('living_roomM', 'living_roomR', 2.45), 
                ('dining_room', 'living_roomL', 1.92), 
                ('dining_room', 'living_roomM', 2.07), 
                ('living_roomR', 'vestibule', 3.10)
            ],
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
        # self.searched_rooms = []
        self.search_path = None
        
        rospy.init_node('search_pubsub', anonymous=True)
        self.current_coords = None
        self.goal_reached = False
        self.goal_object = None

        self.coords_listener = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.coords_listener_callback)
        self.goal_listener = None #! TODO
        self.mission_listener = None #! TODO
        self.goal_publisher = rospy.Publisher('/goal_coords', Point32, queue_size=10)

        self.state = 'dormant'
        self.run()

    def run(self):
        while True:
            # if self.state == 'dormant':
            #     pass
            if self.state == 'ready_for_path':
                self.find_optimal_search_path()
            elif self.state == 'ready_for_next_stop':
                next_stop = self.search_path.pop(0)
                next_stop_coords = self.house_rooms['coordinates'][next_stop]
                self.go(next_stop_coords)
            
            # elif self.state == 'defining_path':
            #     pass
            # elif self.state == 'travelling':
            #     pass
            
            elif self.state == 'ready_for_turn':
                # call turn()
                pass
            elif self.state == 'turning':
                pass
    
    # def reset(self):
    #     self.searched_rooms = []

    def find_closest_room(self, coords, searched_rooms):
        x1, y1 = coords
        min_d, closest_room = float('inf'), None
        for room in self.house_rooms['coordinates']:
            if room not in searched_rooms:
                x2, y2 = self.house_rooms['coordinates'][room]
                d = math.sqrt((x2[0]-x1[0])**2 + (y2[1]-y1[1])**2)
                if d < min_d:
                    min_d = d
                    closest_room = room
        return closest_room
    
    def go(self, coords): # publish to /goal_coords (geometry_msgs.msg Point32)
        self.state = 'travelling'
        coords_msg = Point32(x=coords[0], y=coords[1], z=0)
        self.coords_publisher.publish(coords_msg)

    def turn(self):
        self.state = 'turning'
        # if object found, do 'self.goal_reached = True' before returning
        return False # returns True if desired object is found

    def find_optimal_search_path(self):
        if self.current_coords: # if the robot doesn't know its own position yet, this function will be continuously called until it does
            self.state = 'defining_path' # this way, find_optimal_search_path is only called once per mission
            path = []
            starting_coords = self.current_coords

            while len(path) < len(self.house_rooms['coordinates']):
                new_room = self.find_closest_room(starting_coords, path)
                path.append(new_room)
                new_room_coords = self.house_rooms['coordinates'][new_room]
                starting_coords = new_room_coords

            self.search_path = path
        else:
            #! don't know if this works but the intent was to have a 2 second delay between each call to this function
            #! needs testing
            sleep(2)


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

    # def init_listener(self):
    #     rospy.loginfo('INIT SUBSCRIBER !!!!!!!!!!!!!!!!!!!!!!!!!!!!')
    #     sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.listener_callback)
    #     return sub

    def coords_listener_callback(self, data):
        position = data.pose.pose.position
        self.current_coords = (position.x, position.y)
        # rospy.loginfo('CALLBACK CALLBACK CALLBACK')
        # rospy.loginfo(self.__str__())

    def goal_listener_callback(self):
        self.goal_reached = True
        if len(self.search_path == 0):
            self.state = '' #! TODO
        else:
            self.state = 'ready_for_next_stop'

    def mission_listener_callback(self, data):
        #! TODO: setup mission from data
        self.goal_reached = False
        # self.goal_object = ...  
        self.state = 'ready_for_path'

    # def init_publisher(self):
    #     rospy.loginfo('INIT PUBLISHER !!!!!!!!!!!!!!!!!!!!!!!!!!!!')
    #     pub = rospy.Publisher('/goal_coords', Point32, queue_size=10)
    #     return pub

    def __str__(self):
        return "robot position: " + str(self.current_coords)

search = Search()
# search.go((1,1))



# from tree_search import *

# class Rooms(SearchDomain):
#     def __init__(self,connections, coordinates):
#         self.connections = connections
#         self.coordinates = coordinates
#     def actions(self,room):
#         actlist = []
#         for (R1,R2,_) in self.connections:
#             if (R1==room):
#                 actlist += [(R1,R2)]
#             elif (R2==room):
#                actlist += [(R2,R1)]
#         return actlist
#     def result(self,room,action):
#         (R1,R2) = action
#         if R1==room:
#             return R2
#     def cost(self, room, action):
#         (R1,R2) = action
#         if room==R1 or room==R2:
#             for conn in self.connections:
#                 if (conn[0]==R1 and conn[1]==R2) or (conn[0]==R2 and conn[1]==R1):
#                     return conn[2]
#     def heuristic(self, room, goal_room):
#         R1 = self.coordinates[room]
#         R2 = self.coordinates[goal_room]
#         distance = math.sqrt( (R2[0]-R1[0])**2 + (R2[1]-R1[1])**2 ) # d = √[ (x2 − x1)^2 + (y2 − y1)^2 ]
#         return distance
#     def satisfies(self, room, goal_room):
#         return goal_room==room


# house_rooms = {
#     # connections between rooms
#     'connections': [
#         # (R1, R2, distance)
#         ('bedroom1L', 'bedroom1R', 2.52),
#         ('bedroom1R', 'hall1', 1.58),
#         ('bedroom2T', 'bedroom2B', 2.28),
#         ('bedroom2B', 'hall1', 2.51), 
#         ('bedroom2B', 'hall2', 3.53), 
#         ('bedroom3T', 'bedroom3B', 1.70), 
#         ('bedroom3B', 'hall2', 2.16), 
#         ('toilet1', 'hall2', 1.40), 
#         ('vestibule', 'hall2', 2.20), 
#         ('toilet2', 'vestibule', 1.37), 
#         ('dining_room', 'hall1', 2.45), 
#         ('dining_room', 'kitchen', 2.59), 
#         ('dining_room', 'terraceT', 3.03), 
#         ('terraceT', 'terraceB', 4.61), 
#         ('terraceB', 'living_roomL', 1.60), 
#         ('living_roomL', 'living_roomM', 2.04), 
#         ('living_roomM', 'living_roomR', 2.45), 
#         ('dining_room', 'living_roomL', 1.92), 
#         ('dining_room', 'living_roomM', 2.07), 
#         ('living_roomR', 'vestibule', 3.10)
#     ],
#     # coordinates of each room
#     'coordinates': {
#         # R: (x, y)
#         'bedroom1L': (-7.37, 3.09),
#         'bedroom1R': (-4.87, 2.80),
#         'bedroom2T': (-2.80, 4.96),
#         'bedroom2B': (-3.42, 2.77),
#         'bedroom3T': (-0.51, 4.48),
#         'bedroom3B': (-0.59, 2.78),
#         'hall1': (-5.49, 1.35),
#         'hall2': (-0.62, 0.62),
#         'vestibule': (0.29, -1.38),
#         'kitchen': (-2.28, -0.88),
#         'dining_room': (-4.87, -1.02),
#         'toilet1': (0.76, 0.86),
#         'toilet2': (1.66, -1.37),
#         'living_roomL': (-5.92, -2.63),
#         'living_roomM': (-3.89, -2.84),
#         'living_roomR': (-1.64, -3.81),
#         'terraceT': (-7.17, 0.96),
#         'terraceB': (-7.15, -3.65)
#     }
# }

# searched_rooms = []

# def find_closest_room(coords, searched):
#     x1, y1 = coords
#     min_d, closest_room = float('inf'), None
#     for room in house_rooms['coordinates']:
#         if room not in searched:
#             x2, y2 = house_rooms['coordinates'][room]
#             d = math.sqrt((x2[0]-x1[0])**2 + (y2[1]-y1[1])**2)
#             if d < min_d:
#                 min_d = d
#                 closest_room = room
#     return closest_room

# def go(coords):
#     pass

# def turn():
#     return False # returns True if desired object is found

# def search(current_coords):
#     start_room = find_closest_room(current_coords)
#     start_room_coords = house_rooms['coordinates'][start_room]
#     go(start_room_coords)
#     found = turn()
#     searched_rooms.append(start_room)

#     # while there are still rooms to search
#     while not found and len(searched_rooms) < len(house_rooms['coordinates']):
#         next_room = find_closest_room(start_room_coords, searched_rooms)
#         if next_room:
#             next_room_coords = house_rooms['coordinates'][next_room]
#             go(next_room_coords)
#             found = turn()
#             searched_rooms.append(next_room)
#         else:
#             break
    



















# # test stuff
# p = SearchProblem(house_rooms, 'bedroom', 'kitchen')
# t = SearchTree(p, 'breadth')
# print(t.search())


# def search_path(R1, R2, strategy):
#     """ Obtain path between rooms R1 and R2 using a given strategy. """
#     my_prob = SearchProblem(house_rooms, R1, R2)
#     my_tree = SearchTree(my_prob)
#     my_tree.strategy = strategy
#     return my_tree.search()