#!/usr/bin/env python
from __future__ import print_function

import numpy as np

import rospy
from rospy.numpy_msg import numpy_msg
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.pcl_helper import *


def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    """Helper function to create a yaml friendly dictionary from ROS messages"""
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)

    return yaml_dict


def send_to_yaml(yaml_filename, dict_list):
    """Helper function to output to yaml file"""
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)


def callback(object_list):
    # TODO: Initialize variables

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # Read parameters

    # The format of 'object_list' is:
    # [{'group': 'red', 'name': 'sticky_notes'},
    #  {'group': 'red', 'name': 'book'},
    #  {'group': 'green', 'name': 'snacks'}]
    object_params_list = rospy.get_param('/object_list')
    # The format of 'dropbox' is:
    # [{'group': 'red', 'name': 'left', 'position': [0, 0.71, 0.605]},
    #  {'group': 'green', 'name': 'right', 'position': [0, -0.71, 0.605]}]
    dropbox_params = rospy.get_param('/dropbox')

    # Make a new data structure for future look-up
    object_information = dict()
    for object_params in object_params_list:
        if object_params['group'] == 'red':
            object_information[object_params['name']] = \
                (dropbox_params[0]['name'], dropbox_params[0]['position']) 
        elif object_params['group'] == 'green':
            object_information[object_params['name']] = \
                (dropbox_params[1]['name'], dropbox_params[1]['position']) 
        else:
            print("{}: Unknown group!".format(object_params['group']))

    yaml_dict_list = []
    for obj in object_list.objects:
        # Create a list of dictionaries for later output to yaml format
        test_scene_num = Int32()
        test_scene_num.data = 3

        object_name = String()
        object_name.data = str(obj.label)

        centroid = np.mean(ros_to_pcl(obj.cloud).to_array(), axis=0)[:3]
        pick_pose = Pose()
        pick_pose.position.x = np.asscalar(centroid[0])
        pick_pose.position.y = np.asscalar(centroid[1])
        pick_pose.position.z = np.asscalar(centroid[2])
        pick_pose.orientation.x = 0
        pick_pose.orientation.y = 0
        pick_pose.orientation.z = 0
        pick_pose.orientation.w = 0
        
        arm_name = String()
        arm_name.data = str(object_information[obj.label][0])

        place_pose = Pose()
        place_pose.position.x = object_information[obj.label][1][0]
        place_pose.position.y = object_information[obj.label][1][1]
        place_pose.position.z = object_information[obj.label][1][2]
        place_pose.orientation.x = 0
        place_pose.orientation.y = 0
        place_pose.orientation.z = 0
        place_pose.orientation.w = 0

        yaml_dict_list.append(make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose))

        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # Insert your message variables to be sent as a service request
            resp = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)

            print ("Response: ", resp.success)

        except rospy.ServiceException, e:
            print("Service call failed: %s".format(e))

    send_to_yaml('../config/output_3.yaml', yaml_dict_list)


if __name__ == '__main__':

    rospy.init_node('pick_and_place', anonymous=True)

    pcl_sub = rospy.Subscriber("/detected_objects", DetectedObjectsArray, callback)
    # In rospy, each subscriber has its own thread which handles its 
    # callback functions automatically. 
    rospy.spin()

