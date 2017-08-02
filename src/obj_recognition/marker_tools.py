#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Brandon Kinman

import rospy

from visualization_msgs.msg import Marker

def make_label(text, position, id = 0 ,duration = 5.0, color=[1.0,1.0,1.0]):
    """ Helper function for generating visualization markers.
    
        Args:
            text (str): Text string to be displayed.
            position (list): List containing [x,y,z] positions
            id (int): Integer identifying the label
            duration (rospy.Duration): How long the label will be displayed for
            color (list): List of label color floats from 0 to 1 [r,g,b]
        
        Returns: 
            Marker: A text view marker which can be published to RViz
    """
    marker = Marker()
    marker.header.frame_id = '/world'
    marker.id = id
    marker.type = marker.TEXT_VIEW_FACING
    marker.text = text
    marker.action = marker.ADD
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05
    marker.color.a = 1.0
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.lifetime = rospy.Duration(duration)
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = position[0]
    marker.pose.position.y = position[1]
    marker.pose.position.z = position[2]
    return marker