import matplotlib.colors
import matplotlib.pyplot as plt
import numpy as np
from pcl_helper import *


def rgb_to_hsv(rgb_list):
    rgb_normalized = [1.0*rgb_list[0]/255, 1.0*rgb_list[1]/255, 1.0*rgb_list[2]/255]
    hsv_normalized = matplotlib.colors.rgb_to_hsv([[rgb_normalized]])[0][0]
    return hsv_normalized


def compute_color_histograms(cloud, using_hsv=False):

    numBins = 64
    # Compute histograms for the clusters
    point_colors_list = []

    # Step through each point in the point cloud
    for point in pc2.read_points(cloud, skip_nans=True):
        rgb_list = float_to_rgb(point[3])
        if using_hsv:
            point_colors_list.append(rgb_to_hsv(rgb_list) * 255)
        else:
            point_colors_list.append(rgb_list)

    # Populate lists with color values
    channel_1_vals = []
    channel_2_vals = []
    channel_3_vals = []


    for color in point_colors_list:
        channel_1_vals.append(color[0])
        channel_2_vals.append(color[1])
        channel_3_vals.append(color[2])

    # Compute histograms for the colors in the point cloud
    channel1_hist = np.histogram(channel_1_vals, bins=numBins, range=(0, 256))
    channel2_hist = np.histogram(channel_2_vals, bins=numBins, range=(0, 256))
    channel3_hist = np.histogram(channel_3_vals, bins=numBins, range=(0, 256))



    # Concatenate and normalize the histograms
    hist_features = np.concatenate((channel1_hist[0],channel2_hist[0], channel3_hist[0])).astype(np.float64)
    normed_features = hist_features / np.sum(hist_features)
    return normed_features


def compute_normal_histograms(normal_cloud):
    norm_x_vals = []
    norm_y_vals = []
    norm_z_vals = []
    numBins = 64

    for norm_component in pc2.read_points(normal_cloud,
                                          field_names = ('normal_x', 'normal_y', 'normal_z'),
                                          skip_nans=True):
        norm_x_vals.append(norm_component[0])
        norm_y_vals.append(norm_component[1])
        norm_z_vals.append(norm_component[2])

    # Compute histograms for the normals in the point cloud
    norm1_hist = np.histogram(norm_x_vals, bins=numBins, range=(0, 256))
    norm2_hist = np.histogram(norm_y_vals, bins=numBins, range=(0, 256))
    norm3_hist = np.histogram(norm_z_vals, bins=numBins, range=(0, 256))


    # Concatenate and normalize the histograms
    norm_hist_features = np.concatenate((norm1_hist[0],norm2_hist[0], norm3_hist[0])).astype(np.float64)
    norm_features = norm_hist_features / np.sum(norm_hist_features)
    return norm_features
