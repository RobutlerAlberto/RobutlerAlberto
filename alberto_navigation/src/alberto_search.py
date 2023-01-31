#!/usr/bin/env python3

import math
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped



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
        self.searched_rooms = []
        self.current_coords = None
        self.coords_listener = self.init_listener()
    
    def reset(self):
        self.searched_rooms = []

    def find_closest_room(self, coords):
        x1, y1 = coords
        min_d, closest_room = float('inf'), None
        for room in self.house_rooms['coordinates']:
            if room not in self.searched_rooms:
                x2, y2 = self.house_rooms['coordinates'][room]
                d = math.sqrt((x2[0]-x1[0])**2 + (y2[1]-y1[1])**2)
                if d < min_d:
                    min_d = d
                    closest_room = room
        return closest_room
    
    def go(self, coords):
        pass

    def turn(self):
        return False # returns True if desired object is found

    def search(self):
        if self.current_coords:
            start_room = self.find_closest_room(self.current_coords)
            start_room_coords = self.house_rooms['coordinates'][start_room]
            self.go(start_room_coords)
            found = self.turn()
            self.searched_rooms.append(start_room)

            # while there are still rooms to search
            while not found and len(self.searched_rooms) < len(self.house_rooms['coordinates']):
                next_room = self.find_closest_room(start_room_coords, self.searched_rooms)
                if next_room:
                    next_room_coords = self.house_rooms['coordinates'][next_room]
                    self.go(next_room_coords)
                    found = self.turn()
                    self.searched_rooms.append(next_room)
                else:
                    break
        else:
            print('Robot doesn\'t know its own position yet, wait a bit!')

    def init_listener(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.listener_callback)
        rospy.spin()

    def listener_callback(self, data):
        position = data.pose.pose.position
        self.current_coords = (position.x, position.y)
        rospy.loginfo(self.__str__())

    def __str__(self):
        return "robot position: " + str(self.current_coords)

search = Search()



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