#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from vision.msg import dice_offsets
from cv_bridge import CvBridge, CvBridgeError

class dice_finder:
    def __init__(self):
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(rospy.get_param("~topic_in_name"), Image, self.detect)
        self.pub = rospy.Publisher(rospy.get_param("~topic_out_name"), dice_offsets, queue_size=1)
        self.disable = False
        self.dims = (550, 400) # width, height

    def detect(self, img):
        msg = dice_offsets()
        msg.max_dice_offset.x = 0
        msg.max_dice_offset.y = 0
        msg.min_dice_offset.x = 0
        msg.min_dice_offset.y = 0
        self.pub.publish(msg) 
        
if __name__ == '__main__':
    rospy.init_node('dice_finder', anonymous=True)
    a = dice_finder()
    rospy.spin()

