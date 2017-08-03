#!/usr/bin/env python

import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder

import pickle

from obj_recognition.srv import GetNormals
from obj_recognition.features import compute_color_histograms
from obj_recognition.features import compute_normal_histograms
from visualization_msgs.msg import Marker

from obj_recognition.marker_tools import *
from obj_recognition.msg import DetectedObjectsArray
from obj_recognition.msg import DetectedObject
from obj_recognition.pcl_helper import *

def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Callback function for your subscriber to the cluster array
def pcl_callback(pcl_msg):


    # TODO Classify the clusters! (loop through each detected cluster one at a time)

        # Grab the points for the cluster

        # Compute the associated feature vector

        # Make the prediction

        # Publish a label into RViz

        # Add the detected object to the list of detected objects.

    # Publish the list of detected objects

if __name__ == '__main__':

    # TODO: ROS node initialization

    # TODO: Create Subscribers

    # TODO: Create Publishers

    # TODO: Load Model From disk

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
