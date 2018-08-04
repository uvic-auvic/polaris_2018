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
        self.cannypub = rospy.Publisher("Canny_video", Image, queue_size=1)
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
        #can_img = self.bridge.cv2_to_imgmsg(cv_img, "bgr8")
        #self.cannypub.publish(cv_img)

        #mask = np.ones((self.dims[1]+2, self.dims[0]+2), np.uint8)
        #cv2.floodFill(cv_img, mask, (0,0), 255)
        
        #squashed_height = cv2.reduce(cv_img, 0, cv2.REDUCE_SUM, dtype=cv2.CV_32S)[0]
        #squashed_height.sort(reverse=True)
        
        _, contours, hierarchy = cv2.findContours(cv_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        contours.sort(key=cv2.contourArea, reverse=True)
        contours = contours
        rospy.logerr(len(contours))

        if len(contours) > 10:
            contours = contours[0:10]


        x1 = 0
        x2 = 0
        x_delta = 0

        new_contours = []
        for cnt in contours:
            x,y,w,h = cv2.boundingRect(cnt)
            if h/w > 1.3:
                new_contours.append(cnt)
                if x1 == 0:
                    x1 = x
                else:
                    if abs(x1 - x) > x_delta:
                        x2 = x
                        x_delta = abs(x2-x1)

        if len(new_contours) == 0:
            return
        
        for cnt in new_contours:
            x,y,w,h = cv2.boundingRect(cnt)

            font				   = cv2.FONT_HERSHEY_SIMPLEX
            bottomLeftCornerOfText = (x,y-20)
            fontScale			  = 1
            fontColor			  = (255,255,255)
            lineType			   = 2
            

            cv2.putText(orig_img,str((x1+x2)/2), 
                bottomLeftCornerOfText, 
                font, 
                fontScale,
                fontColor,
                lineType)
        
        center_x = self.dims[0] / 2
        
        if x2 != 0 and (abs(x1-x2)>50):
            x_pos = ((x1+x2)/2)
        else:
            x_pos = center_x

        cv2.line(orig_img, (x_pos, 0), (x_pos, self.dims[1]), 3)

        cv2.drawContours(orig_img, new_contours, -1, (0,0,1), 10)
        it_img = self.bridge.cv2_to_imgmsg(orig_img, "bgr8")
        self.impub.publish(it_img)
        
        # publish position
        
        offset_x = ((x1+x2)/2) - center_x
        msg = offset_position()
        msg.relative_offset_x = -int(100 * (float(offset_x) / center_x))
        self.pub.publish(msg) # return the position as a percentage of the width
        
if __name__ == '__main__':
    rospy.init_node('object_scanner', anonymous=True)
    a = object_scanner()
    rospy.spin()

