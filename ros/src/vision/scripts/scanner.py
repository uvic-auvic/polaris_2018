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
        self.disable = False
        self.dims = (550, 400) # width, height

    # threshold is in terms of pixels
    # img is mutated
    def remove_short_paths(self, img, threshold=10):
        cpy = img.copy()
        height, width = cpy.shape
        for y, row in enumerate(img):
            for x, _ in enumerate(row):
                clear_list = []
                if self.dfs(cpy, height, width, x, y, clear_list) < threshold:
                    # Delete the chain from the original
                    #self.dfs(img, height, width, x, y)
                    for ox, oy in clear_list:
                        img[oy][ox] = 0
        

    # Run a depth first search on the image
    # by default the function will clobber any values that it runs into
    def dfs(self, img, height, width, x, y, clear_list):
        # check if we're out of bounds
        if x < 0 or y < 0 or x >= width or y >= height:
            return 0

        count = 0
        # check if we just broke the chain
        if img[y][x] == 0:
            return count
        else:
            # set it to zero so we don't run into it again
            img[y][x] = 0
            clear_list.append((x,y))

        # Check everything thats to the right, below and diagonally downwards
        count += self.dfs(img, height, width, x+1, y, clear_list)
        count += self.dfs(img, height, width, x-1, y+1, clear_list)
        count += self.dfs(img, height, width, x, y+1, clear_list)
        count += self.dfs(img, height, width, x+1, y+1, clear_list)

        return count
        
    def detect(self, img):
        if self.disable:
            return
        
        msg = offset_position()
        msg.relative_offset_x = 0
        img = self.bridge.imgmsg_to_cv2(img, "bgr8")
        orig_img = cv2.resize(img, self.dims)
        cv_img = cv2.GaussianBlur(orig_img, (9,9), 0)
        cv_img = cv2.medianBlur(cv_img, 5)

        # canny edge detection
        cv_img = cv2.Canny(cv_img, threshold1=75, threshold2=110)
        mask = np.ones((self.dims[1]+2, self.dims[0]+2), np.uint8)
        cv2.floodFill(cv_img, mask, (0,0), 255)
        squashed_height = cv2.reduce(cv_img, 0, cv2.REDUCE_SUM, dtype=cv2.CV_32S)[0]

        # find susbtrings of non-zero items in list
        threshold = 2500
        centers = []
        start = -1

        for i, val in enumerate(squashed_height):
            if val >= threshold:
                if start == -1:
                    start = i
            else:
                if start != -1:
                    height_sum = sum(squashed_height[start:i]) / 255
                    width_sum = i - start
                    gradient = height_sum / width_sum
                    centers.append((height_sum, gradient, start, i))
                    start = -1

        centers = [x for x in centers if x[2] > 2]
        if len(centers) < 2:
            self.impub.publish(self.bridge.cv2_to_imgmsg(orig_img, "bgr8"))
            self.pub.publish(msg)
            return

        centers = sorted(centers, key=lambda x: x[0], reverse=True)
        diff = abs((centers[0][0] / centers[1][0]) - 1)

        # 2 vertical structures not comparable enough to be markers
        if diff > 1:
            self.impub.publish(self.bridge.cv2_to_imgmsg(orig_img, "bgr8"))
            self.pub.publish(msg)
            return

        x = (centers[0][1] + centers[0][2] + centers[1][1] + centers[1][2]) / 4
        # Draw line on image to show position to move towards
        cv2.line(orig_img, (x, 0), (x, self.dims[1]), 3)
        it_img = self.bridge.cv2_to_imgmsg(orig_img, "bgr8")
        self.impub.publish(it_img)
        
        # publish position
        center_x = self.dims[0] / 2
        offset_x = x - center_x
        msg.relative_offset_x = 100 * (float(offset_x) / center_x)
        self.pub.publish(msg) # return the position as a percentage of the width
        
if __name__ == '__main__':
    rospy.init_node('object_scanner', anonymous=True)
    a = object_scanner()
    rospy.spin()

