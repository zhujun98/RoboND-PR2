#!/usr/bin/env python
from __future__ import print_function
import sys

import numpy as np

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.pcl_helper import ros_to_pcl 
import yaml


def send_to_yaml(target_objects, detected_objects, test_scene_num): 
    """Output the pick and place requests to yaml file
    
    :param target_objects: dictionary
        key: name (label)
        value: (side of the dropbox [left/right], coordinates of the dropbox)
    :param detected_objects: dictionary
        key: name of the object
        value: centroid coordinates (x, y, z) of the object
    :param test_scene_num: int
        Test scene number.
    """
    yaml_dict_list = []
    for label in target_objects.keys():
        try:
            centroid =  detected_objects[label]
        except KeyError:
            continue

        scene_num = Int32()
        scene_num.data = test_scene_num 

        object_name = String()
        object_name.data = str(label)

        pick_pose = Pose()
        pick_pose.position.x = np.asscalar(centroid[0])
        pick_pose.position.y = np.asscalar(centroid[1])
        pick_pose.position.z = np.asscalar(centroid[2])
        pick_pose.orientation.x = 0
        pick_pose.orientation.y = 0
        pick_pose.orientation.z = 0
        pick_pose.orientation.w = 0
        
        arm_name = String()
        arm_name.data = str(target_objects[label][0])

        place_pose = Pose()
        place_pose.position.x = target_objects[label][1][0]
        place_pose.position.y = target_objects[label][1][1]
        place_pose.position.z = target_objects[label][1][2]
        place_pose.orientation.x = 0
        place_pose.orientation.y = 0
        place_pose.orientation.z = 0
        place_pose.orientation.w = 0

        yaml_dict = dict() 
        yaml_dict["test_scene_num"] = scene_num.data
        yaml_dict["arm_name"]  = arm_name.data
        yaml_dict["object_name"] = object_name.data
        yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
        yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)

        yaml_dict_list.append(yaml_dict)

    data_dict = {"object_list": yaml_dict_list}
    filename = "../config/output_" + str(test_scene_num) + ".yaml"
    with open(filename, 'w') as fp:
        yaml.dump(data_dict, fp, default_flow_style=False)

    print("Data saved in {}".format(filename))


class PickAndPlace(object):
    """"""
    def __init__(self):
        """Initialization"""
        self._is_initialized = False

        if len(sys.argv) == 1:
            self._test_scene_num = 0
        else:
            self._test_scene_num = int(sys.argv[1])

        self.target_objects = self._generate_target_objects()

    @staticmethod
    def _generate_target_objects():
        """Generate pick-up objects information
    
        The format of target_objects:
              key: name (label)
            value: (side of the dropbox [left/right], coordinates of the dropbox)
        """
        # The format of 'object_list' is:
        # [{'group': 'red', 'name': 'sticky_notes'},
        #  {'group': 'red', 'name': 'book'},
        #  {'group': 'green', 'name': 'snacks'}]
        object_params_list = rospy.get_param('/object_list')
        # The format of 'dropbox' is:
        # [{'group': 'red', 'name': 'left', 'position': [0, 0.71, 0.605]},
        #  {'group': 'green', 'name': 'right', 'position': [0, -0.71, 0.605]}]
        dropbox_params = rospy.get_param('/dropbox')
    
        target_objects = dict()
        for object_params in object_params_list:
            if object_params['group'] == 'red':
                target_objects[object_params['name']] = \
                    (dropbox_params[0]['name'], dropbox_params[0]['position']) 
            elif object_params['group'] == 'green':
                target_objects[object_params['name']] = \
                    (dropbox_params[1]['name'], dropbox_params[1]['position']) 
            else:
                print("{}: Unknown group!".format(object_params['group']))
    
        return target_objects

    def callback(self, object_list):
        # TODO: Initialize variables
    
        # TODO: Rotate PR2 in place to capture side tables for the collision map
    
        # Read parameters
    
        # Make a dictionary of detected object to look-up
        #   key: name of the object
        # value: centroid coordinates of the object
        detected_objects = dict()
        for obj in object_list.objects:
            # The format of DetectObject:
            #   string label
            #   sensor_msgs/PointCloud2 cloud
            detected_objects[str(obj.label)] = np.mean(
                ros_to_pcl(obj.cloud).to_array(), axis=0)[:3]

        # ------------------------------------------------------------- 
        # Generate the yaml file, as required by the project
        # ------------------------------------------------------------- 
        if self._is_initialized == False:
            send_to_yaml(self.target_objects, detected_objects, self._test_scene_num) 
            self._is_initialized = True

#         # Wait for 'pick_place_routine' service to come up
#         rospy.wait_for_service('pick_place_routine')
#     
#         try:
#             pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)
#             
#             if len(self.target_objects.keys()) > 0:
#                 label, value = self.target_objects.iteritems()[0]
#                  
#             # Insert your message variables to be sent as a service request
#             resp = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)
#     
#             print ("Response: ", resp.success)
#     
#         except rospy.ServiceException, e:
#             print("Service call failed: %s".format(e))
 

if __name__ == '__main__':
    rospy.init_node('pick_and_place', anonymous=True)

    pick_and_place = PickAndPlace()
    pcl_sub = rospy.Subscriber("/detected_objects", DetectedObjectsArray, 
                               pick_and_place.callback)
    
    # In rospy, each subscriber has its own thread which handles its 
    # callback functions automatically. 
    rospy.spin()

