#!/usr/bin/env python3

import random
import argparse
import rospy
import rospkg
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion

def main():
    parser = argparse.ArgumentParser(description='Object Spawner')
    parser.add_argument('-o', '--object', type=str, required=False, help= 'Object name, can be cube or sphere.')
    parser.add_argument('-c', '--color', type=str, required=False, help= 'Object color, can be blue, green, orange, yellow or purple.')
    parser.add_argument('-r', '--room', type=str, required=False, help='Room where to spawn object. Can be living_room, kitchen, large_bedroom, small_bedroom, office or outside.')
    args = parser.parse_args()

    rospy.init_node('insert_object',log_level=rospy.INFO)

    # get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('alberto_description') + '/description/models/'


    #Add here mode poses
    placements = []
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

    
    model_names = ['cube_blue', 'cube_dark_blue', 'cube_green', 'cube_orange', 'cube_purple', 'cube_red', 'cube_yellow', 
                        'sphere_blue', 'sphere_green', 'sphere_orange', 'sphere_purple', 'sphere_red', 'sphere_yellow']

    if args.room:
        model_name = args.object + '_' + args.color
        for i in placements:
            if i['room'] == args.room:
                model_placement = i
    else:
        # Add here several models. All should be added to the robutler_description package
        model_name = random.choice(model_names)
        model_placement = random.choice(placements)
        

    f = open( package_path + model_name + '/model.sdf' ,'r')
    sdff = f.read()

    rospy.wait_for_service('gazebo/spawn_sdf_model')
    spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)

    name = model_name + '_in_' + model_placement['place'] + '_of_' + model_placement['room']
    print(name)
    spawn_model_prox(name, sdff, model_name, model_placement['pose'], "world")

if __name__ == '__main__':
    main()