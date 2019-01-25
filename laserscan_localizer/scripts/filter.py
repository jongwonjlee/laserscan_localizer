#!/usr/bin/env python 
"""
Depreciated while using intensity_filter in "laser_filters" package 
Watch intensity_filtered data. Cutoff frequency is mentioned below.
"""
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan


def filter(msg):
    cutoff_freq = 150
    global counter

    if counter % 15 == 0:
        index_mtx = np.arange(0, 811,1).reshape(-1,1)
        angle_mtx = np.arange(-135.0, +135.1, 270./810).reshape(-1,1)
        range_mtx = np.asarray(msg.ranges).reshape(-1,1)
        intensity_mtx = np.asarray(msg.intensities).reshape(-1,1)
        
        thres = np.where(intensity_mtx >= cutoff_freq)[0]
        index_thres = index_mtx[thres]
        angle_thres = angle_mtx[thres]
        range_thres = range_mtx[thres]
        intensity_thres = intensity_mtx[thres]
        
        concat_mtx = np.concatenate([index_thres, angle_thres, intensity_thres, range_thres], axis=1)
        print "      index      angle       intensity       range\n", concat_mtx
    
    counter += 1


if __name__ == '__main__':
    counter = 0
    rospy.init_node("filter")
    rospy.Subscriber("scan", LaserScan, filter)

    rospy.spin()
