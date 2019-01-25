#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud
import std_msgs.msg


if __name__ == '__main__':

    rospy.init_node('pixel_to_coordinate_calculator')
    cloud_pub = rospy.Publisher(
        'pointcloud_debug',
        PointCloud,
        queue_size=10
    )
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        debug_pointcloud = PointCloud()
        debug_pointcloud.header = std_msgs.msg.Header()
        debug_pointcloud.header.stamp = rospy.Time.now()
        debug_pointcloud.header.frame_id = "laser"

        number_of_pixels = 10
        # create an empty list of correct size
        debug_pointcloud.points = [None] * number_of_pixels

        # fill up pointcloud with points where x value changes but y and z are all 0
        for p in xrange(0, number_of_pixels):
            debug_pointcloud.points[p] = Point(p, 0, 0)

        # now publish the debug pointcloud
        cloud_pub.publish(debug_pointcloud)
        rate.sleep()

    rospy.spin()

