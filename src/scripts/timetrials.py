#! /usr/bin/env python3

"""Subscribes to raw image topic, processes data and finds road.  Follows road, publishes to cmd_vel topic
Adaped from ROS example on compressed images http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber
"""
__author__ =  'Simon Haller <simon.haller at uibk.ac.at>'
__version__=  '0.1'
__license__ = 'BSD'
# Python libs
import sys
import math
# numpy and scipy
import numpy as np
# OpenCV
import cv2
import time

# Ros libraries
import roslib
import rospy
from cv_bridge import CvBridge

# Ros Messages
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import os
# We do not use cv_bridge it does not support CompressedImage in python
# from cv_bridge import CvBridge, CvBridgeError

BOTTOM_ROW_OF_IMAGE = 719
SCAN_ROW = BOTTOM_ROW_OF_IMAGE - 250
STEERING_CENTER = 640
COLUMNS_IN_IMAGE = 1279
VERBOSE=False


move = Twist()
counter = 0


class robot_driving:
    """Class that subsribes to image feed"""

    def __init__(self):
        '''Initialize ros subscriber'''
        self.steering_val = STEERING_CENTER
        self.prev_steering_val = STEERING_CENTER
        self.prev_state = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
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
        # move.linear.x = 0.5
        # self.publisher.publish(move)


        if VERBOSE :
            print ("/rrbot/camera/image_raw")

          
    def locate_road(self, row_num, mask):
        """@brief scans a row of image for the road, returns pixel num of some point on the road (counting up
        from left hand side of the screen)
        @param row_num the number of the row being scanned (top to bottom)
        @param mask the mask to be scanned
        @retval value of road pixel, measured from left of screen
        """
        left_road_pixel = self.__scan_row_for_road(row_num,mask,True)
        right_road_pixel = self.__scan_row_for_road(row_num,mask,False)
        if left_road_pixel != -1 and right_road_pixel != -1:
            return math.floor((left_road_pixel + right_road_pixel)/2)
        return -1
    
    def __scan_row_for_road(self,row_num,mask, from_left = True):
        image = mask
        road_pixel_counter = 0
        for i in range(COLUMNS_IN_IMAGE): #loop through row, getting pixel val for every col
            if from_left:
                colour = image[row_num,i]
            else:
                colour = image[row_num,COLUMNS_IN_IMAGE - i]
            if(colour < 225): #if Colour is close to white it is road
                road_pixel_counter += 1
            else: #if coulour is not white than reset counter
                road_pixel_counter = 0
            if road_pixel_counter > 4: #if greater than 10 consecutive columns are road, return pixel num
                if from_left:
                    return i
                else:
                    return COLUMNS_IN_IMAGE - i
        
        return -1 # value returned if there is no road on the screen

    def get_steering_val(self):
        """
        @brief modifies prev_steering_val, move
        uses steering value to find the track, then follow it, updating move each time it is called
        """
        global move
        if(self.steering_val == -1 and self.prev_steering_val == -1): #robot initialization - face road
            move.linear.x = 0
            move.angular.z = 0.05
        elif(self.prev_steering_val == -1) : #first time seeing road, begin driving forward
            self.prev_steering_val = self.steering_val
            move.linear.x = 0.2
            move.angular.z = 0
        elif(self.steering_val != -1): #if seeing road, set move command based on difference of road position from center
            move.linear.x = 0.2
            move.angular.z = -(self.steering_val-STEERING_CENTER)/150
            self.prev_steering_val = self.steering_val
        else:
            move.linear.x = 0.0 #if road is lost, rotate in direction road was last seen until road is found again
            move.angular.z = -(self.prev_steering_val-STEERING_CENTER)/150


    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and road is detected, steering val is set'''

        global counter, move
        cv_image = self.bridge.imgmsg_to_cv2(ros_data, 'bgr8')
        blur_image = cv2.GaussianBlur(cv_image, (7,7), 0)
        blur_image = cv2.GaussianBlur(blur_image, (7,7), 0)

        greyscale = cv2.cvtColor(blur_image, cv2.COLOR_BGR2GRAY)

        # Define the lower and upper bounds for black (low brightness)
        lower_black = np.array([0])  # Lower bound
        upper_black = np.array([180])  # Upper bound 

        black_mask = cv2.inRange(greyscale, lower_black, upper_black)

        
        line_position = self.locate_road(SCAN_ROW,black_mask)
        self.steering_val = line_position
        self.get_steering_val()
        self.publisher.publish(move)
        cv2.circle(cv_image, (line_position, SCAN_ROW), 5, (0,0,255), -1)

        if line_position == -1:
            state = self.prev_state
        else:
            state = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            line_position = math.floor(line_position/COLUMNS_IN_IMAGE*len(state))
            state[int(line_position)] = 1
        
        self.prev_state = state

        #display states with vertical lines separating them
        for i in range(1,len(state)):
            cv2.line(cv_image, (int(i/len(state)*COLUMNS_IN_IMAGE),0), (int(i/len(state)*COLUMNS_IN_IMAGE),800), (0,255,0), 1)
        #show state array in small text on top of image
        cv2.putText(cv_image, str(state), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
        #get q values for each action in state
        #0 is fwd, 1 is left, 2 is right
        #display q values on image
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

        if counter == 2000:
            rospy.sleep(3)
            move.linear.x = 0
            self.publisher.publish(move)
            self.publisher2.publish('TeamName,password,-1,NA')
            rospy.sleep(1)
            os.kill(os.getpid(),9)

        counter += 1
        print(self.steering_val)
        
        #self.subscriber.unregister()

def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('robot_driving', anonymous=True)
    ic = robot_driving()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
