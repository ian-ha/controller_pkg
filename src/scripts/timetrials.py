#! /usr/bin/env python3

"""Subscribes to raw image topic, processes data and finds road.  Follows road, publishes to cmd_vel topic
Adaped from ROS example on compressed images http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber
"""
__author__ =  'Simon Haller <simon.haller at uibk.ac.at>'
__version__=  '0.1'
__license__ = 'BSD'
# Python libs
import sys

# numpy and scipy
import numpy as np
# OpenCV
import cv2
import time

# Ros libraries
import roslib
import rospy
from cv_bridge import CvBridge

import rospy
import cv2
import sys
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

# Ros Messages
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import os
# We do not use cv_bridge it does not support CompressedImage in python
# from cv_bridge import CvBridge, CvBridgeError

BOTTOM_ROW_OF_IMAGE = 719
COLUMNS_IN_IMAGE = 800
VERBOSE=False

STEERING_CENTER = 400
steering_val = -1
prev_steering_val = -1

move = Twist()
counter = 0

def get_steering_val():
   """
    @brief modifies prev_steering_val, move
   uses steering value to find the track, then follow it, updating move each time it is called
   """
   global steering_val, prev_steering_val
   if(steering_val == -1 and prev_steering_val == -1): #robot initialization - face road
      move.linear.x = 0
      move.angular.z = 2
   elif(prev_steering_val == -1) : #first time seeing road, begin driving forward
      prev_steering_val = steering_val
      move.linear.x = 1
      move.angular.z = 0
   elif(steering_val != -1): #if seeing road, set move command based on difference of road position from center
      move.linear.x = 1
      move.angular.z = -(steering_val-STEERING_CENTER)/50
      prev_steering_val = steering_val
   else:
      move.linear.x = 0.1 #if road is lost, rotate in direction road was last seen until road is found again
      move.angular.z = -(prev_steering_val-STEERING_CENTER)/50
      
    
      

def scan_row_for_road(row_num, mask):
    """@brief scans a row of image for the road, returns pixel num of some point on the road (counting up
from left hand side of the screen)
@param row_num the number of the row being scanned (top to bottom)
@param mask the mask to be scanned
@retval value of road pixel, measured from left of screen
"""
    image = mask
    road_pixel_counter = 0
    for i in range(COLUMNS_IN_IMAGE): #loop through row, getting pixel val for every col
        colour = image[row_num,i]
        if(colour > 200): #if Colour is close to white it is road
            road_pixel_counter += 1
        else: #if coulour is not white than reset counter
            road_pixel_counter = 0
        if road_pixel_counter > 10: #if greater than 10 consecutive columns are road, return pixel num
            return i
    
    return -1 # value returned if there is no road on the screen

class line_follower:

#   def __init__(self):
#     self.image_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

#     self.bridge = CvBridge()
#     self.image_sub = rospy.Subscriber("/rrbot/camera1/image_raw", Image, self.callback)

    def __init__(self):
        '''Initialize ros subscriber'''
        self.bridge = CvBridge()

        # subscribed Topic
        self.subscriber = rospy.Subscriber("/R1/pi_camera/image_raw",
            Image, self.callback,  queue_size = 1)

        self.publisher = rospy.Publisher("/R1/cmd_vel",Twist, queue_size=1)
        '''initialize ros publisher'''

        self.publisher2 = rospy.Publisher("/score_tracker",String, queue_size=1)
        rospy.sleep(1)

        self.publisher2.publish('TeamName,password,0,NA')
        global move
        move.linear.x = 0.5
        self.publisher.publish(move)


        if VERBOSE :
            print ("/rrbot/camera/image_raw")


    def callback(self, data):
        # convert from ROS image to CV2
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)



        # @brief calculate road center
        (rows,cols,channels) = cv_image.shape

        
        
        # Identify the row 2 pixels above the bottom edge
        row_index_2 = cv_image.shape[0] - 2 - 1

        # Find pixel values for that row
        row_pixels_2 = cv_image[row_index_2, :, :]

        # Calculate brightness for each pixel in the row
        brightness_values_2 = np.sum(row_pixels_2[:, :3], axis=1)

        # Find the coordinates of the leftmost and rightmost dark spots
        dark_pixel_indices_2 = np.where(brightness_values_2 < 268)[0]
        middle_spot=0
        if len(dark_pixel_indices_2) > 0:
            leftmost_dark_spot = dark_pixel_indices_2[0]
            rightmost_dark_spot = dark_pixel_indices_2[-1]
            middle_spot = (leftmost_dark_spot + rightmost_dark_spot) // 2
            middle_coordinates = (middle_spot, row_index_2)

            # Draw a large red dot at the middle spot using OpenCV's circle function
            cv2.circle(cv_image, middle_coordinates, 5, [0, 0, 255], -1)  # Red circle with r = 5
            cv2.imshow("Image road", cv_image)
            cv2.waitKey(3)
        else:
            middle_coordinates = None

        x_mid_of_frame = cols // 2

        mid_frame = (x_mid_of_frame, row_index_2)
        print(middle_coordinates)
        print(mid_frame)
        print(row_pixels_2[600])

        
        move = Twist()

        if(x_mid_of_frame-middle_spot>0):
            move.linear.x = 0.25
            move.angular.z = (x_mid_of_frame-middle_spot)/80
        else:
            move.linear.x = 0.25
            move.angular.z = (x_mid_of_frame-middle_spot)/80
        try:
            self.publisher.publish(move)
        except CvBridgeError as e:
            print(e)



def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('image_feature', anonymous=True)
    ic = line_follower()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

