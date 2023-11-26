#! /usr/bin/env python3

"""
This program subscribes to a raw image topic and detects quadrilaterals in the image using OpenCV without edge detection.
"""

import sys
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np

VERBOSE = False

class ImageDisplay:
    """Class that subscribes to image feed, displays it, and attempts to detect quadrilaterals directly in the image"""

    def __init__(self):
        '''Initialize ros subscriber'''
        self.bridge = CvBridge()
        # subscribed Topic
        self.subscriber = rospy.Subscriber("/R1/pi_camera/image_raw", Image, self.callback, queue_size=1)
        if VERBOSE:
            print("Subscribed to /R1/pi_camera/image_raw")

    def callback(self, ros_data):
        '''Callback function of subscribed topic. Here images get converted and quadrilaterals detected'''
        cv_image = self.bridge.imgmsg_to_cv2(ros_data, 'bgr8')

        # Convert the image to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Thresholding the grayscale image to get better results
        _, thresh = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY)

        # Find contours
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Iterate over contours and approximate to polygons
        for cnt in contours:
            # Approximate the contour to a polygon
            epsilon = 0.02 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)

            # If the polygon has 4 vertices, we assume it's a quadrilateral
            if len(approx) == 4:
                cv2.drawContours(cv_image, [approx], 0, (0, 255, 0), 3)  # Draw in green
                for vertex in approx:
                    cv2.circle(cv_image, tuple(vertex[0]), 5, (0, 0, 255), -1)  # Vertices in blue

        cv2.imshow("Image with Detected Quadrilaterals", cv_image)
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
