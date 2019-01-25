#!/usr/bin/env python 
import numpy as np
from matplotlib import pyplot as plt
import rospy
from sensor_msgs.msg import LaserScan
import time


def plot_data(msg):
    global counter
    if counter % 5 == 0:
        stamp = msg.header.stamp
        time = stamp.secs + stamp.nsecs * 1e-9

        plt.gcf().clear()

        angle_mtx = np.arange(-135.0, +135.1, 270./810).tolist()

        range_mtx = list(msg.ranges)
        intensity_mtx = list(msg.intensities)

        plt.figure(1)
        plt.subplot(211)
        plt.xlabel('Angle[deg]')
        plt.ylabel('Range[m]')
        plt.title('Range and Intensity')
        plt.axis([-135, +135, 0, 35])
        range_graph = plt.plot(angle_mtx, range_mtx, linewidth=2,color = 'r')
        plt.subplot(212)
        plt.xlabel('Angle[deg]')
        plt.ylabel('Intensity')
        plt.axis([-135, +135, 0, 250])
        intensity_graph = plt.plot(angle_mtx, intensity_mtx, linewidth=2,color = 'b')

	plt.figure(1).canvas.draw()
        plt.figure(1).canvas.flush_events()
        counter = 1

    else: 
        counter += 1

if __name__ == '__main__':
    counter = 0
    rospy.init_node("plotter")
    rospy.Subscriber("scan", LaserScan, plot_data)
    plt.ion()
    plt.show()

    rospy.spin()

    plt.close('all')
