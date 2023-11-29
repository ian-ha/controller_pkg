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

        # Apply the filter for blue color
        blue_channel = cv_image[:,:,0]
        red_channel = cv_image[:,:,2]
        green_channel = cv_image[:,:,1]
        mask = (blue_channel >= 1.8 * red_channel) & (blue_channel >= 1.8 * green_channel)
        filtered_image = np.zeros_like(cv_image)
        filtered_image[mask] = [255, 255, 255]

        # Convert to grayscale
        gray = cv2.cvtColor(filtered_image, cv2.COLOR_BGR2GRAY)

        # Find contours for blue quadrilateral
        contours, _ = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Detect the largest blue quadrilateral
        max_area = 0
        max_quad = None
        for cnt in contours:
            perimeter = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * perimeter, True)
            if len(approx) == 4 and cv2.contourArea(approx) > max_area:
                max_area = cv2.contourArea(approx)
                max_quad = approx

        if max_quad is not None:
            # Create a mask for the blue quadrilateral
            mask_quad = np.zeros_like(gray)
            cv2.fillPoly(mask_quad, [max_quad.astype(int)], 255)

            # Adjust the mask to exclude blue quadrilaterals
            non_blue_mask = (blue_channel < 1.8 * red_channel) | (blue_channel < 1.8 * green_channel)
            combined_mask = mask_quad & non_blue_mask

            # Find contours for non-blue quadrilaterals within the blue quadrilateral
            contours, _ = cv2.findContours(combined_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            # Detect the largest non-blue quadrilateral within the blue quadrilateral
            max_non_blue_area = 0
            max_non_blue_quad = None
            for cnt in contours:
                perimeter = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.02 * perimeter, True)
                if len(approx) == 4 and cv2.contourArea(approx) > max_non_blue_area:
                    max_non_blue_area = cv2.contourArea(approx)
                    max_non_blue_quad = approx

            # Apply perspective transformation to the largest non-blue quadrilateral
            if max_non_blue_quad is not None:
                area_non_blue_quad = cv2.contourArea(max_non_blue_quad)
                if area_non_blue_quad > 15000:
                    max_non_blue_quad = self.order_points(max_non_blue_quad[:, 0, :])
                    pts1 = np.float32(max_non_blue_quad)
                    pts2 = np.float32([[0, 0], [600, 0], [600, 400], [0, 400]])
                    matrix = cv2.getPerspectiveTransform(pts1, pts2)
                    result = cv2.warpPerspective(cv_image, matrix, (600, 400))

                    # Take a picture of 'result' and save it
                    cv2.imwrite("non_blue_quadrilateral_image_{}.jpg".format(rospy.Time.now()), result)
                    print("Picture of the non-blue quadrilateral taken.")
                    cv2.imshow("Perspective Transformation", result)

            # Draw the contour of the non-blue quadrilateral
            if max_non_blue_quad is not None:
                cv2.drawContours(cv_image, [max_non_blue_quad.astype(int)], 0, (0, 0, 255), 3)

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
