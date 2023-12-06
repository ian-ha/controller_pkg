#! /usr/bin/env python3

import sys
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np
import random
from std_msgs.msg import String

VERBOSE = False
IMG_AREA_THRESHOLD1 = 19000
IMG_AREA_THRESHOLD2 = 20000
TIME_THRESHOLD = 3
DATA_COLLECTION = False

class ImageDisplay:
    def __init__(self):
        self.bridge = CvBridge()
        self.subscriber = rospy.Subscriber("/R1/pi_camera/image_raw", Image, self.callback, queue_size=1)
        self.sign_pub = rospy.Publisher('sign_detected', String, queue_size=2)
        self.last_image_time = rospy.Time()
        self.plate_num = 0
        self.image_captured = False
        self.img_filepath = r'/home/fizzer/competition_images/'
        self.img_temp_path = r'/home/fizzer/ros_ws/src/controller_pkg/src/plateImages/'
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
                current_time = rospy.Time.now()
                if area_non_blue_quad > IMG_AREA_THRESHOLD2 and not self.image_captured and (current_time - self.last_image_time).to_sec() > TIME_THRESHOLD:
                    max_non_blue_quad = self.order_points(max_non_blue_quad[:, 0, :])
                    pts1 = np.float32(max_non_blue_quad)
                    pts2 = np.float32([[0, 0], [600, 0], [600, 400], [0, 400]])
                    matrix = cv2.getPerspectiveTransform(pts1, pts2)
                    result = cv2.warpPerspective(cv_image, matrix, (600, 400))

                    # Take a picture of 'result' and save it

                    self.plate_num+=1 # plate number

                    # Plates Directory below:

                    # '/home/fizzer/ros_ws/src/2023_competition/enph353/enph353_gazebo/scripts/plates.csv'

                    plate_content = self.get_plate_content(self.plate_num, '/home/fizzer/ros_ws/src/2023_competition/enph353/enph353_gazebo/scripts/plates.csv')
                    if plate_content:
                        filename = "{}.jpg".format(plate_content)
                        #save to competition_images filepath
                        if DATA_COLLECTION:
                            cv2.imwrite(self.img_filepath + filename, result)
                        cv2.imwrite(self.img_temp_path + filename, result)
                        #print("Picture of {} taken.".format(filename))
                        #print(area_non_blue_quad)
                        print("hq sign detected")
                    else:
                        print("No plate content found for plate number {}".format(self.plate_num))

                    cv2.imshow("Perspective Transformation", result)
                    self.image_captured = True
                    self.last_image_time = current_time
                    self.sign_pub.publish("Sign Detected")
                if area_non_blue_quad > IMG_AREA_THRESHOLD1 and not self.image_captured and (current_time - self.last_image_time).to_sec() > TIME_THRESHOLD:
                    max_non_blue_quad = self.order_points(max_non_blue_quad[:, 0, :])
                    pts1 = np.float32(max_non_blue_quad)
                    pts2 = np.float32([[0, 0], [600, 0], [600, 400], [0, 400]])
                    matrix = cv2.getPerspectiveTransform(pts1, pts2)
                    result = cv2.warpPerspective(cv_image, matrix, (600, 400))
                    self.sign_pub.publish("Sign Detected")
                    print("lq sign")

                    # Take a picture of 'result' and save it

                    self.plate_num+=1 # plate number

                    # Plates Directory below:

                    # '/home/fizzer/ros_ws/src/2023_competition/enph353/enph353_gazebo/scripts/plates.csv'

                    plate_content = self.get_plate_content(self.plate_num, '/home/fizzer/ros_ws/src/2023_competition/enph353/enph353_gazebo/scripts/plates.csv')
                    if plate_content:
                        filename = "{}.jpg".format(plate_content)
                        #save to competition_images filepath
                        if DATA_COLLECTION:
                            cv2.imwrite(self.img_filepath + filename, result)
                        cv2.imwrite(self.img_temp_path + filename, result)
                        #print("Picture of {} taken.".format(filename))
                        #print(area_non_blue_quad)
                    else:
                        print("No plate content found for plate number {}".format(self.plate_num))

                    cv2.imshow("Perspective Transformation", result)
                    self.image_captured = True
                    self.last_image_time = current_time
                elif area_non_blue_quad <= IMG_AREA_THRESHOLD1:
                    self.image_captured = False

            # Draw the contour of the non-blue quadrilateral
            if max_non_blue_quad is not None:
                cv2.drawContours(cv_image, [max_non_blue_quad.astype(int)], 0, (0, 0, 255), 3)

        # Display the edge-detected image
        #cv2.imshow("Masked Image", filtered_image)
        
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
    
    def get_plate_content(self, plate_num, file_path):
        with open(file_path, 'r') as file:
            lines = file.readlines()
            if 0 <= plate_num - 1 < len(lines):
                content = lines[plate_num - 1].strip() # Adjust for zero-indexing
                return content.replace(',', '').replace(' ', '')
            else:
                return None

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
