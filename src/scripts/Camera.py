#! /usr/bin/env python3

import sys
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np

VERBOSE = False

class ImageDisplay:
    def __init__(self):
        self.bridge = CvBridge()
        self.subscriber = rospy.Subscriber("/R1/pi_camera/image_raw", Image, self.callback, queue_size=1)
        if VERBOSE:
            print("Subscribed to /R1/pi_camera/image_raw")

    def callback(self, ros_data):
        cv_image = self.bridge.imgmsg_to_cv2(ros_data, 'bgr8')

        # Apply the filter
        blue_channel = cv_image[:,:,0]
        red_channel = cv_image[:,:,2]
        green_channel = cv_image[:,:,1]
        mask = (blue_channel >= 1.8 * red_channel) & (blue_channel >= 1.8 * green_channel)
        filtered_image = np.zeros_like(cv_image)
        filtered_image[mask] = [255, 255, 255]

        # Convert to grayscale
        gray = cv2.cvtColor(filtered_image, cv2.COLOR_BGR2GRAY)

        # Find contours
        contours, _ = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Detect the largest quadrilateral
        max_area = 0
        max_quad = None
        for cnt in contours:
            perimeter = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * perimeter, True)
            if len(approx) == 4 and cv2.contourArea(approx) > max_area:
                max_area = cv2.contourArea(approx)
                max_quad = approx

        if max_quad is not None:
            max_quad = self.order_points(max_quad[:, 0, :])  # Reorder points

            # Draw the contour
            cv2.drawContours(cv_image, [max_quad.astype(int)], 0, (0, 255, 0), 3)

            # Perspective transformation
            pts1 = np.float32(max_quad)
            pts2 = np.float32([[0, 0], [500, 0], [500, 500], [0, 500]])
            matrix = cv2.getPerspectiveTransform(pts1, pts2)
            result = cv2.warpPerspective(cv_image, matrix, (500, 500))

            # Display the transformed image
            cv2.imshow("Perspective Transformation", result)

        # Display the edge-detected image
        cv2.imshow("Masked Image", filtered_image)
        cv2.waitKey(3)

    def order_points(self, pts):
        rect = np.zeros((4, 2), dtype="float32")
        s = pts.sum(axis=1)
        diff = np.diff(pts, axis=1)
        rect[0] = pts[np.argmin(s)]
        rect[2] = pts[np.argmax(s)]
        rect[1] = pts[np.argmin(diff)]
        rect[3] = pts[np.argmax(diff)]
        return rect

def main(args):
    rospy.init_node('image_display', anonymous=True)
    id = ImageDisplay()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image display module")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
