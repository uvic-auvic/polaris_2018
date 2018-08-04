#!/usr/bin/env python
import rospy
import cv2
import math
import numpy as np
from sensor_msgs.msg import Image
from vision.msg import dice_offsets
from cv_bridge import CvBridge, CvBridgeError

class dice_finder:
    def __init__(self):
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(rospy.get_param("~topic_in_name"), Image, self.detect)
        self.pub = rospy.Publisher(rospy.get_param("~topic_out_name"), dice_offsets, queue_size=1)
        self.impub = rospy.Publisher(rospy.get_param("~topic_out_name") + "_video", Image, queue_size=1)
        self.dims = (550, 400) # width, height
        self.seen = False

    def detect(self, img):
        msg = dice_offsets()
        msg.max_dice_offset.x_offset = 0
        msg.max_dice_offset.y_offset = 0
        msg.min_dice_offset.x_offset = 0
        msg.min_dice_offset.y_offset = 0

        frame = self.bridge.imgmsg_to_cv2(img, "bgr8")

        # Our operations on the frame come here
        frame = cv2.resize(frame, self.dims)
        y,x = frame.shape[:2]
        mid_point = (int(x/2),int(y/2))

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        orignal = frame.copy()
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        gray = cv2.medianBlur(gray, 5)
        gray = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 3.5)

        kernel = np.ones((3, 3), np.uint8)
        gray = cv2.erode(gray, kernel, iterations=1)
        gray = cv2.dilate(gray, kernel, iterations=1)

        _, contours, hierarchy = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        contours_list = []
        area_list = []
        center_list = []

        for contour in contours:
            approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)
            area = cv2.contourArea(contour)
            (x, y), radius = cv2.minEnclosingCircle(contour)
            cal_area = radius * radius * math.pi
            area2 = 1.6 * area
            if ((len(approx) > 8) & (len(approx) < 23) & (200 > area > 50) & (cal_area < area2)):
                contours_list.append(contour)
                center = (int(x), int(y))
                center_list.append(center)
                area_list.append(area)

        distance_list = []
        min_list1 = []
        min_list2 = []

        for center_1 in center_list:
            x1,y1 = center_1
            for center_2 in center_list:
                x2,y2 = center_2
                if ((x1 != x2) & (y1 != y2)):
                    distance = math.hypot(x2-x1, y2-y1)
                    distance_list.append(distance)

        for center_1 in center_list:
            x1,y1 = center_1
            for center_2 in center_list:
                x2,y2 = center_2
                distance = math.hypot(x2-x1, y2-y1)
                if ((x1 != x2) & (y1 != y2)):
                    if (distance < (3*min(distance_list))):
                        if center_1 not in min_list1 and center_2 not in min_list2:
                            min_list1.append(center_1)
                            min_list2.append(center_2)


        clusterA_x = []
        clusterA_y = []
        clusterB_x = []
        clusterB_y = []

        for x1,y1 in min_list1:
            for x2,y2 in min_list2:
                if ((x1 != x2) & (y1 != y2)):
                    x = abs(x1-x2)
                    y = abs(y1-y2)
                    if x > 100 or y >70:
                        if x1 not in clusterA_x and x1 not in clusterB_x and y1 not in clusterA_y and y1 not in clusterB_y:
                            if x2 not in clusterA_x and x2 not in clusterB_x and y2 not in clusterA_y and y2 not in clusterB_y:
                                if x1 > np.mean(clusterA_x)+30:
                                    clusterB_x.append(x1)
                                elif y1 > np.mean(clusterA_y)+20:
                                    clusterB_y.append(y1)
                                else:
                                    clusterA_x.append(x1)
                                    clusterA_y.append(y1)
                                    clusterB_x.append(x2)
                                    clusterB_y.append(y2)

        if clusterA_x and clusterB_y and clusterA_y and clusterB_x is not 0:
            max_x = max(clusterB_x) - float(mid_point[0])
            max_y = max(clusterB_y) - float(mid_point[1])
            min_x = min(clusterA_x) - float(mid_point[0])
            min_y = min(clusterA_y) - float(mid_point[1])

            msg.max_dice_offset.x_offset = int(100 * (max_x / mid_point[0]))
            msg.max_dice_offset.y_offset = int(100 * (max_y / mid_point[1]))
            msg.min_dice_offset.x_offset = int(100 * (min_x / mid_point[0]))
            msg.min_dice_offset.y_offset = int(100 * (min_y / mid_point[1]))

            a_center = (msg.min_dice_offset.x_offset,msg.min_dice_offset.y_offset)
            b_center = (msg.max_dice_offset.x_offset,msg.max_dice_offset.y_offset)

            cv2.line(orignal,mid_point,a_center,(255,0,0),5)
            cv2.line(orignal,mid_point,b_center,(0,0,255),5)
            self.seen =True
        cv2.drawContours(orignal, contours_list, -1, (255, 0, 0), 1)

        if self.seen and msg.max_dice_offset.x_offset == 0 and msg.max_dice_offset.y_offset == 0:
            pass
        else:
            self.pub.publish(msg)
            self.seen = True
        it_img  = self.bridge.cv2_to_imgmsg(orignal, "bgr8")
        self.impub.publish(it_img)

        
if __name__ == '__main__':
    rospy.init_node('dice_finder', anonymous=True)
    a = dice_finder()
    rospy.spin()

