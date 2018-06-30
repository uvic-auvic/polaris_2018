#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from scipy.interpolate import UnivariateSpline
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def create_LUT_8UC1(x, y):
    spl = UnivariateSpline(x, y)
    return spl(xrange(256))

class image_classifier:
    def __init__(self):
        # ROS handles
        topic_name = "/video/%s" % rospy.get_param("~topic_name")
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(topic_name, Image, self.convert)
        self.pub = rospy.Publisher("/video/corrected", Image, queue_size=10)

        # Open CV configutations
        invGamma = 1.0 / rospy.get_param('~gamma')
        self.lookup_table = np.array(
            [((i / 255.0) ** invGamma) * 255 for i in np.arange(0, 256)]).astype("uint8")
        self.incr_ch_lut = create_LUT_8UC1([0, 64, 128, 192, 256], [0, 70, 140, 210, 256])
        self.decr_ch_lut = create_LUT_8UC1([0, 64, 128, 192, 256], [0, 30, 80, 120, 192])

        # internal state
        self.count = 0

    def convert(self, img):
        # process 60% of the images
        self.count += 1
        if self.count % 2:
            return
        elif self.count == 5:
            self.count = 0

        cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        #rs_image = cv2.resize(cv_image, (0, 0), None, 0.75, 0.75)
        bright_img = self.color_temperature_convert(cv_image)
        avg_img = self.average_and_subtract(bright_img)
        
        # insert tensorflow here
        it_img = self.bridge.cv2_to_imgmsg(avg_img, "bgr8")
        self.pub.publish(it_img)

    def average_and_subtract(self, img, bdiv=0.35, gdiv=0.15, rdiv=0.0):
        average_color_per_row = np.average(img, axis=0)
        average_color = np.average(average_color_per_row, axis=0)
        avg_color = [average_color[0] * bdiv, average_color[1] * gdiv, average_color[2] * rdiv]
        width, height, _ = img.shape
        average_color_img = np.array([[average_color]*height]*width, np.uint8)
        return cv2.subtract(img, average_color_img)

    def color_temperature_convert(self, img):
        b, g, r = cv2.split(img)
        b = cv2.LUT(b, self.decr_ch_lut).astype(np.uint8)
        #g = cv2.LUT(g, self.decr_ch_lut).astype(np.uint8)
        r = cv2.LUT(r, self.incr_ch_lut).astype(np.uint8)
        return cv2.merge((b, g, r))

    def color_correct(self, img):
        return cv2.LUT(img, self.lookup_table)

if __name__ == '__main__':
    rospy.init_node("classifier", anonymous=True)
    ic = image_classifier()
    rospy.spin()
    cv2.destroyAllWindows()

