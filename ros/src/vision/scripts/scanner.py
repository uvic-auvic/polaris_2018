#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from vision.msg import offset_position
from cv_bridge import CvBridge, CvBridgeError

class object_scanner:
    def __init__(self):
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(rospy.get_param("~topic_in_name"), Image, self.detect)
        self.pub = rospy.Publisher(rospy.get_param("~topic_out_name"), offset_position, queue_size=1)
        #self.impub = rospy.Publisher(rospy.get_param("~topic_out_name"), Image, queue_size=1)
        self.disable = False
        self.dims = (550, 400) # width, height

    def detect(self, img):
        if self.disable:
            return
        
        img = self.bridge.imgmsg_to_cv2(img, "bgr8")
        orig_img = cv2.resize(img, self.dims)
        cv_img = cv2.GaussianBlur(orig_img, (9,9), 0)
        cv_img = cv2.medianBlur(cv_img, 5)

        # canny edge detection
        cv_img = cv2.Canny(cv_img, threshold1=75, threshold2=110)
        dst = cv2.reduce(cv_img, 0, cv2.REDUCE_SUM, dtype=cv2.CV_32S)[0]

        # find susbtrings of non-zero items in list
        threshold = 3000
        centers = []
        start = -1

        for i, val in enumerate(dst):
            if val >= threshold:
                if start == -1:
                    start = i
            else:
                if start != -1:
                    centers.append((sum(dst[start:i]), start, i))
                    start = -1

        if len(centers) < 2:
            return

        centers = sorted(centers, key=lambda x: x[0], reverse=True)
        diff = abs((centers[0][0] / centers[1][0]) - 1)

        # 2 vertical structures not comparable enough to be markers
        if diff > 1:
            return

        x = (centers[0][1] + centers[0][2] + centers[1][1] + centers[1][2]) / 4
        # Draw line on image to show position to move towards
        #cv2.line(orig_img, (x, 0), (x, self.dims[1]), 3)
        #it_img = self.bridge.cv2_to_imgmsg(orig_img, "bgr8")
        #self.impub.publish(it_img)
        
        # publish position
        center_x = self.dims[0] / 2
        offset_x = x - center_x
        msg = offset_position()
        msg.relative_offset_x = 100 * (float(offset_x) / center_x)
        self.pub.publish(msg) # return the position as a percentage of the width
        
if __name__ == '__main__':
    rospy.init_node('object_scanner', anonymous=True)
    a = object_scanner()
    rospy.spin()

