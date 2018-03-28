#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

def callback(data):
    sys.stderr.write("got pcd maps. going to create temporary tf.launch file\n")
    points = pc2.read_points(data, field_names = ("x", "y", "z"), skip_nans=True)
    counter = len(points)
    xsum = sum(map(lambda x: x[0], points))
    ysum = sum(map(lambda x: x[1], points))
    zsum = sum(map(lambda x: x[2], points))
    print "<launch>"
    print "<node pkg=\"tf\" type=\"static_transform_publisher\" name=\"world_to_map\" args=\"%f %f %f 0 0 0 /world /map 10 \" /> " %(-1* xsum/counter, -1*ysum/counter, -1*zsum/counter)
    print "</launch>"
    sys.stderr.write("Finished!\n")
    rospy.signal_shutdown("end")
        # print " x : %f  y: %f  z: %f" %(p[0],p[1],p[2])


def listener():
    rospy.init_node('listener', anonymous =True)
    rospy.Subscriber("points_map", PointCloud2, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()


