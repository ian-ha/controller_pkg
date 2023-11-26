#! /usr/bin/env python3

"""
This program subscribes to a raw image topic, applies a specific filter to the image, and then displays it using OpenCV.
"""

import sys
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np

VERBOSE = False

class ImageDisplay:
    """Class that subscribes to image feed and displays the filtered image"""

    def __init__(self):
        '''Initialize ros subscriber'''
        self.bridge = CvBridge()
        # subscribed Topic
        self.subscriber = rospy.Subscriber("/R1/pi_camera/image_raw", Image, self.callback, queue_size=1)
        if VERBOSE:
            print("Subscribed to /R1/pi_camera/image_raw")

    def callback(self, ros_data):
        '''Callback function of subscribed topic. Here images get converted and filtered'''
        cv_image = self.bridge.imgmsg_to_cv2(ros_data, 'bgr8')

        # Apply the filter
        blue_channel = cv_image[:,:,0]
        red_channel = cv_image[:,:,2]
        green_channel = cv_image[:,:,1]

        # Create a mask where the blue channel is at least 1.8 times the red and green channels
        mask = (blue_channel >= 1.8 * red_channel) & (blue_channel >= 1.8 * green_channel)

        # Create a new image, white where the mask is True, black elsewhere
        filtered_image = np.zeros_like(cv_image)
        filtered_image[mask] = [255, 255, 255]

        cv2.imshow("Filtered Image", filtered_image)
        cv2.waitKey(3)

def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('image_display', anonymous=True)
    id = ImageDisplay()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image display module")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
