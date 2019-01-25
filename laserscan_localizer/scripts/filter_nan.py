#!/usr/bin/env python 
"""
Depreciated while using intensity_filter in "laser_filters" package 
Watch intensity_filtered data. Cutoff frequency is mentioned below.
"""
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan

def print_row(mtx):
    if not np.isnan(mtx[3]):
        print mtx

def filter(msg):
    global counter

    if counter % 15 == 0:
        index_mtx = np.arange(0, 811,1).reshape(-1,1)
        angle_mtx = np.arange(-135.0, +135.1, 270./810).reshape(-1,1)
        range_mtx = np.asarray(msg.ranges).reshape(-1,1)
        intensity_mtx = np.asarray(msg.intensities).reshape(-1,1)
        concat_mtx = np.concatenate([index_mtx, angle_mtx, intensity_mtx, range_mtx], axis=1)
        print "      index      angle       intensity       range"
        np.apply_along_axis( print_row, axis=1, arr=concat_mtx)
    
    
    counter += 1


if __name__ == '__main__':
    counter = 0
    rospy.init_node("filter")
    rospy.Subscriber("scan_filtered", LaserScan, filter)

    rospy.spin()
