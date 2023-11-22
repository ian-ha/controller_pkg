#! /usr/bin/env python3

"""
This program subscribes to a raw image topic and displays the image using OpenCV.
"""

import sys
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2

VERBOSE=False

class ImageDisplay:
    """Class that subscribes to image feed and displays it"""

    def __init__(self):
        '''Initialize ros subscriber'''
        self.bridge = CvBridge()
        # subscribed Topic
        self.subscriber = rospy.Subscriber("/R1/pi_camera/image_raw", Image, self.callback, queue_size=1)
        if VERBOSE:
            print("Subscribed to /R1/pi_camera/image_raw")

    def callback(self, ros_data):
        '''Callback function of subscribed topic. Here images get converted and displayed'''
        cv_image = self.bridge.imgmsg_to_cv2(ros_data, 'bgr8')

        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Apply threshold
        threshold_value = 100  # You can adjust this value
        _, thresh_image = cv2.threshold(gray_image, threshold_value, 255, cv2.THRESH_BINARY)

        cv2.imshow("Thresholded Image", thresh_image)
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
