#!/usr/bin/env python 
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
import sys


def observer(msg, idx):

    global counter

    if counter % 15 == 0:
        index_mtx = np.arange(0, 811,1).reshape(-1,1)
        angle_mtx = np.arange(-135.0, +135.1, 270./810).reshape(-1,1)
        range_mtx = np.asarray(msg.ranges).reshape(-1,1)
        intensity_mtx = np.asarray(msg.intensities).reshape(-1,1)
        
        concat_mtx = np.concatenate([index_mtx, angle_mtx, intensity_mtx, range_mtx], axis = 1)
        
        scope = concat_mtx[idx-3:idx+4, :]
        print "    index    angle  intensity  range\n", scope
    
    counter += 1

if __name__ == '__main__':

    myargv = rospy.myargv(argv = sys.argv)
    if len(myargv) > 2:
        NameError("no sufficient arguments")
    else:
        counter = 0

        float_formatter = lambda x: "%.4f" % x
        np.set_printoptions(formatter={'float_kind':float_formatter})
        
        rospy.init_node("observer")
        rospy.Subscriber("scan", LaserScan, observer, int(myargv[1]))
        rospy.spin()

