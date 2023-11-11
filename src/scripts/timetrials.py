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
      move.linear.x = 2.5
      move.angular.z = 0
   elif(steering_val != -1): #if seeing road, set move command based on difference of road position from center
      move.linear.x = 2.5
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


class image_feature:
    """Class that subsribes to image feed"""

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


    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and road is detected, steering val is set'''

        global counter, move

        if counter == 1:
            rospy.sleep(3)
            move.linear.x = 0
            self.publisher.publish(move)
            self.publisher2.publish('TeamName,password,-1,NA')
            rospy.sleep(1)
            os.kill(os.getpid(),9)

        if counter == 0:
            move.linear.x = 0.5
            self.publisher.publish(move)
            counter += 1


        
        

        
        #self.subscriber.unregister()

def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('image_feature', anonymous=True)
    ic = image_feature()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)