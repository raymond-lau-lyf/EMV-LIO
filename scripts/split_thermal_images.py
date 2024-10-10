#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class ImageSplitter:
    def __init__(self):
        # Initialize node
        rospy.init_node('image_splitter', anonymous=True)

        # Create a CvBridge object
        self.bridge = CvBridge()

        # Subscribe to the merged image topic
        self.image_sub = rospy.Subscriber("/thermal_cam1/image_raw", Image, self.image_callback)

        # Publishers for the two split images
        self.image_pub_left = rospy.Publisher("/thermal_cam1/image_raw8", Image, queue_size=10)
        self.image_pub_right = rospy.Publisher("/thermal_cam1/image_raw16", Image, queue_size=10)

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")  # Assuming grayscale image

            # Check image dimensions
            if cv_image.shape[1] != 1280 or cv_image.shape[0] != 512:
                rospy.logwarn("Received image has incorrect dimensions!")
                return

            # Split the image into two 640x512 images
            left_image = cv_image[:, :640]
            right_image = cv_image[:, 640:]

            # Convert back to ROS Image messages
            image_msg_left = self.bridge.cv2_to_imgmsg(left_image, "mono8")
            image_msg_right = self.bridge.cv2_to_imgmsg(right_image, "mono8")
            
            image_msg_right.header = msg.header 
            image_msg_left.header = msg.header

            # Publish the split images
            self.image_pub_left.publish(image_msg_left)
            self.image_pub_right.publish(image_msg_right)

        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")

if __name__ == '__main__':
    try:
        splitter = ImageSplitter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
