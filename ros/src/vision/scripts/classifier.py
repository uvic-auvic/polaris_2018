#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from inference import detector


class image_classifier:
    def __init__(self):
        self.tf = detector()
        # internal state
        self.count = 0
        self.enable = True

        # ROS handles
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(rospy.get_param("~topic_name"), Image, self.classify)
        self.pub = rospy.Publisher("/video/detection", Image, queue_size=10)


    def classify(self, img):
        cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        # process 1/3 images
        if not self.count and self.enable:
            self.count += 1
        else:
            self.count = 0
            return

        # scale the image down so that it is faster to process
        height = cv_image.shape[0]
        height_scaler = 300.0 / height
        rs_image = cv2.resize(cv_image, (0, 0), None, height_scaler, height_scaler)
        rgb_image = cv2.cvtColor(rs_image, cv2.COLOR_BGR2RGB)

        # Use tensorflow session to run detection
        self.tf.detect(rgb_image)

        detected_img = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
        it_img = self.bridge.cv2_to_imgmsg(detected_img, "bgr8")
        self.pub.publish(it_img)

if __name__ == '__main__':
    rospy.init_node("classifier", anonymous=True)
    ic = image_classifier()
    rospy.spin()

