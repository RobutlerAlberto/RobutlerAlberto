#!/usr/bin/env python3

import random
import rospy
import rospkg
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Int16

rospy.init_node('insert_object',log_level=rospy.INFO)

rospy.Subscriber("/active_mission_ID", Int16, mission_listener_callback)

# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()
package_path = rospack.get_path('alberto_description') + '/description/models/'


#Add here mode poses
placements = []
# placements.append({'pose':Pose(position=Point(x=-5.69, y=4.37, z=0.6), orientation=Quaternion(x=0,y=0,z=0,w=1)),
#               'room':'large_bedroom', 'place': 'bed'})
# placements.append({'pose':Pose(position=Point(x=-7.33, y=5.29, z=0.58), orientation=Quaternion(x=0,y=0,z=0,w=1)),
#               'room':'large_bedroom', 'place': 'bedside_cabinet'})
placements.append({'pose':Pose(position=Point(x=-2.96, y=-3.60, z=0), orientation=Quaternion(x=0,y=0,z=0,w=1)),
              'room':'living_room', 'place': 'sofa'})
placements.append({'pose':Pose(position=Point(x=-4.95, y=-3.60, z=0), orientation=Quaternion(x=0,y=0,z=1,w=0.04)),
              'room':'living_room', 'place': 'table'})
placements.append({'pose':Pose(position=Point(x=-6.06, y=-1.96, z=0), orientation=Quaternion(x=0,y=0,z=0,w=1)),
              'room':'kitchen', 'place': 'table'})
placements.append({'pose':Pose(position=Point(x=-4.95, y=-3.60, z=0), orientation=Quaternion(x=0,y=0,z=1,w=0.04)),
              'room':'living_room', 'place': 'table'})
placements.append({'pose':Pose(position=Point(x=-2.24, y=-0.8, z=0), orientation=Quaternion(x=0,y=0,z=1,w=1)),
              'room':'kitchen', 'place': 'countertop'})
placements.append({'pose':Pose(position=Point(x=-5.97, y=4.23, z=0), orientation=Quaternion(x=0,y=0,z=0,w=1)),
              'room':'large_bedroom', 'place': 'bed'})
placements.append({'pose':Pose(position=Point(x=-1.80, y=3.68, z=0), orientation=Quaternion(x=0,y=0,z=0,w=1)),
              'room':'small_bedroom', 'place': 'fitment'})
placements.append({'pose':Pose(position=Point(x=0.26, y=4.89, z=0), orientation=Quaternion(x=0,y=0,z=0,w=1)),
              'room':'office', 'place': 'floor'})
placements.append({'pose':Pose(position=Point(x=-7.15, y=-5.08, z=0), orientation=Quaternion(x=0,y=0,z=0,w=1)),
              'room':'outside', 'place': 'floor'})


# Add here several models. All should be added to the robutler_description package
model_names = ['cube_blue', 'cube_dark_blue', 'cube_green', 'cube_orange', 'cube_purple', 'cube_red', 'cube_yellow', 
               'sphere_blue', 'sphere_green', 'sphere_orange', 'sphere_purple', 'sphere_red', 'sphere_yellow']

# Random choosing of one model
model_name = random.choice(model_names)

# Model open
f = open( package_path + model_name + '/model.sdf' ,'r')
sdff = f.read()

def mission_listener_callback(mission_id):

    # Here waits for the spawn to be called
    # rospy.wait_for_service('gazebo/spawn_sdf_model')
    if mission_id == 28:
        spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
    
    
        model_placement = random.choice(placements)
        name = model_name + '_in_' + model_placement['place'] + '_of_' + model_placement['room']
        spawn_model_prox(name, sdff, model_name, model_placement['pose'], "world")