#!/usr/bin/env python

import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder

import pickle


from obj_recognition.features import compute_color_histograms
from obj_recognition.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from obj_recognition.srv import GetNormals

from obj_recognition.marker_tools import *
from obj_recognition.msg import DetectedObjectsArray
from obj_recognition.msg import DetectedObject
from obj_recognition.msg import SegmentedClustersArray
from obj_recognition.pcl_helper import *


def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Callback function for your subscriber to the cluster array
def pcl_callback(pcl_msg):


    # TODO Classify the clusters!

    # declare holders for the labels and objects
    detected_objects_labels = []
    detected_objects = []


    for index, pcl_cloud in enumerate(pcl_msg.clusters):

        # Extract histogram features
        chists = compute_color_histograms(pcl_cloud, using_hsv=True)
        normals = get_normals(pcl_cloud)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))

        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        pcl_pointCloud = ros_to_pcl(pcl_cloud)
        label_pos = list(pcl_pointCloud[0])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = pcl_cloud
        detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Publish the list of detected objects
    # This is the output you'll need to complete the upcoming project!
    detected_objects_pub.publish(detected_objects)


if __name__ == '__main__':

    rospy.init_node('obj_recognition_node')

    # Create Subscribers
    sub = rospy.Subscriber('obj_recognition/pcl_clusters' , SegmentedClustersArray, pcl_callback )
    detected_objects_pub = rospy.Publisher('/detected_objects', DetectedObjectsArray , queue_size=1)
    object_markers_pub = rospy.Publisher('/object_markers', Marker , queue_size=1)


    # Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    while not rospy.is_shutdown():
        rospy.spin()
