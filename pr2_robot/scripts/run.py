#!/usr/bin/env python
from __future__ import print_function
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


def get_normals(cloud):
    """Helper function to get surface normals"""
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)

    return get_normals_prox(cloud).cluster


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


def pcl_callback(pcl_msg):
    """Callback function for your Point Cloud Subscriber"""
    # Convert ROS msg to PCL data
    pcl_data = ros_to_pcl(pcl_msg)
    
    # Voxel Grid Downsampling
    vox = pcl_data.make_voxel_grid_filter()
    leaf_size = 0.005
    vox.set_leaf_size(leaf_size, leaf_size, leaf_size)

    pcl_data_filtered = vox.filter()

    # PassThrough Filter
    passthrough = pcl_data_filtered.make_passthrough_filter()
    filter_axis = 'z'
    passthrough.set_filter_field_name (filter_axis)
    axis_min = 0.6
    axis_max = 1.1
    passthrough.set_filter_limits (axis_min, axis_max)

    pcl_data_filtered = passthrough.filter()

    passthrough = pcl_data_filtered.make_passthrough_filter()
    filter_axis = 'y'
    passthrough.set_filter_field_name (filter_axis)
    axis_min = -0.55
    axis_max = 0.55
    passthrough.set_filter_limits (axis_min, axis_max)

    pcl_data_filtered = passthrough.filter()

    # RANSAC Plane Segmentation
    seg = pcl_data_filtered.make_segmenter()

    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)

    inliers, coefficients = seg.segment()

    # Extract inliers and outliers
    cloud_table = pcl_data_filtered.extract(inliers, negative=False)
    cloud_objects = pcl_data_filtered.extract(inliers, negative=True)

    # Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()

    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.02)
    ec.set_MinClusterSize(100)
    ec.set_MaxClusterSize(25000)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()

    # Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []
    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                            white_cloud[indice][1],
                                            white_cloud[indice][2],
                                            rgb_to_float(cluster_color[j])])

    # Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(cloud_objects) 
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    # Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)
    
    # classification

    detected_objects_labels = []
    detected_objects = []
    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster
        pcl_cluster = cloud_objects.extract(pts_list)
        # convert the cluster from pcl to ROS using helper function
        ros_cluster_cloud = pcl_to_ros(pcl_cluster)    

        # Extract histogram features
        color_hists = compute_color_histograms(ros_cluster_cloud, 
                                               using_hsv=True)
        normal_hists = compute_normal_histograms(get_normals(ros_cluster_cloud))

        feature = np.concatenate((color_hists, normal_hists))
                
        # Make the prediction, retrieve the label for the result
        # and add it to detected_objects_labels list
        prediction = clf.predict(scaler.transform(feature.reshape(1, -1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label, label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster_cloud
        detected_objects.append(do)
        

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass


def pr2_mover(object_list):
    """function to load parameters and request PickPlace service"""
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
    for obj in object_list:
        # Create a list of dictionaries for later output to yaml format
        test_scene_num = Int32()
        test_scene_num.data = 1

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
        # rospy.wait_for_service('pick_place_routine')

        # try:
          #   pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # TODO: Insert your message variables to be sent as a service request
            # resp = pick_place_routine(TEST_SCENE_NUM, OBJECT_NAME, WHICH_ARM, PICK_POSE, PLACE_POSE)

            # print ("Response: ",resp.success)

        # except rospy.ServiceException, e:
            # print "Service call failed: %s"%e

    send_to_yaml('../config/output_1.yaml', yaml_dict_list)


if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('clustering', anonymous=True)

    # Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2,
                               pcl_callback, queue_size=1)

    # Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)

    object_markers_pub = rospy.Publisher("/object_markers", Marker,
                                         queue_size=1)
    detected_objects_pub = rospy.Publisher("/deleted_objects",
                                           DetectedObjectsArray, queue_size=1)

    # Load Model From disk
    model = pickle.load(open('../../sensor_stick/scripts/model.pkl', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()

