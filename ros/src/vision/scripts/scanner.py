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
        self.impub = rospy.Publisher(rospy.get_param("~topic_out_name") + "_video", Image, queue_size=1)
        self.dims = (550, 400) # width, height
        
    def detect(self, img):
        cv2_img = self.bridge.imgmsg_to_cv2(img, "bgr8")
        orig_img = cv2.resize(cv2_img, self.dims)
        cv_img = cv2.GaussianBlur(orig_img, (13,13), 0)
        cv_img = cv2.medianBlur(cv_img, 5)

        # canny edge detection
        cv_img = cv2.Canny(cv_img, threshold1=75, threshold2=110)
        #mask = np.ones((self.dims[1]+2, self.dims[0]+2), np.uint8)
        #cv2.floodFill(cv_img, mask, (0,0), 255)
        squashed_height = cv2.reduce(cv_img, 0, cv2.REDUCE_SUM, dtype=cv2.CV_32S)[0]

        # 
        msg = offset_position()
        msg.relative_offset_x = 0
        
        # find susbtrings of non-zero items in list
        threshold = 2300
        centers = []
        start = -1

        for i, val in enumerate(squashed_height):
            if val >= threshold:
                if start == -1:
                    start = i
            else:
                if start != -1:
                    height_sum = sum(squashed_height[start:i])
                    gradient = height_sum / ((i - start) ** 3)
                    if gradient > 4000:
                        centers.append((height_sum, gradient, start, i))
                    start = -1

        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_GRAY2BGR)
        if len(centers) < 2:
            self.impub.publish(img)
            self.pub.publish(msg)
            return

        centers = sorted(centers, key=lambda x: x[0], reverse=True)
        diff = abs((centers[0][0] / centers[1][0]) - 1)

        # 2 vertical structures not comparable enough to be markers
        if diff > 1:
            self.impub.publish(img)
            self.pub.publish(msg)
            return

        x = (centers[0][-2] + centers[0][-1] + centers[1][-2] + centers[1][-1]) / 4
        # Draw line on image to show position to move towards
        cv2.line(orig_img, (x, 0), (x, self.dims[1]), 3)
        it_img = self.bridge.cv2_to_imgmsg(orig_img, "bgr8")
        self.impub.publish(it_img)
        
        # publish position
        center_x = self.dims[0] / 2
        offset_x = x - center_x
        msg.relative_offset_x = int(100 * (float(offset_x) / center_x))
        self.pub.publish(msg) # return the position as a percentage of the width
        
if __name__ == '__main__':
    rospy.init_node('object_scanner', anonymous=True)
    a = object_scanner()
    rospy.spin()

