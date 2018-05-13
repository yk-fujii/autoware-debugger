#!/usr/bin/env python

import rospy

from std_msgs.msg import String

log = []

def callback_text(data):
    global log

def listener():
    rospy.init_node('listener', anonymous =True)
    rospy.Subscriber("decision_maker/operator_help_text", String, callback_text)

    print "time-sec,nsec, atacc2, imu_x, pfwc, pmc, sp1, vxrr,vxrl,srqang, est2"
    rospy.spin()

if __name__ == '__main__':
    listener()


