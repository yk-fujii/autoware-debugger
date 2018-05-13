#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:
   def __init__(self):
       self.image_pub = rospy.Publisher("image_topic_2",Image)
       self.bridge = CvBridge()

   def talker(self):
       im = cv2.imread('/tmp/state.png')
       if not im is None:
           try:
               self.image_pub.publish(self.bridge.cv2_to_imgmsg(im, "bgr8"))
           except CvBridgeError as e:
               print(e)

def main(args):
   ic = image_converter()
   rospy.init_node('image_bridger', anonymous=True)
   while not rospy.is_shutdown():
       ic.talker()
       rospy.sleep(0.1)
   cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
