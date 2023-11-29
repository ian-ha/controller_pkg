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
WALL_ROW = BOTTOM_ROW_OF_IMAGE - 400
STEERING_CENTER = 640
COLUMNS_IN_IMAGE = 1279
GRASS_ROW = BOTTOM_ROW_OF_IMAGE - 275
GRASS_2 = BOTTOM_ROW_OF_IMAGE - 200
VERBOSE=False
<<<<<<< HEAD


move = Twist()
counter = 0

=======
NORMAL_DRIVE = 0
PEDESTRIAN = 1
TRUCK_LOOP = 2
OFFROAD = 3
MOUNTAIN = 4
GRASS_ROAD = 5 

move = Twist()
counter = 0
state_machine = NORMAL_DRIVE
ROBOT_SPEED = 0.38
seen_purple_lines = 0
wall_seen = False
last_purple = time.time()
>>>>>>> d5496e0f9f77218e0d4d88c17072e485ebea95de

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
        self.publisher.publish(move)


        if VERBOSE :
            print ("/rrbot/camera/image_raw")

        
    def check_for_purple(self, image):
        cur_time = time.time()
        global last_purple
        delta_t = cur_time - last_purple
        lower_purple = np.array([200, 0, 200])
        upper_purple = np.array([255, 15, 255])
        purple_pixels = 0
        purple_mask = cv2.inRange(image, lower_purple, upper_purple)
        cv2.imshow("purple", purple_mask)
        for i in range(COLUMNS_IN_IMAGE):
            for j in range(GRASS_ROW-2, GRASS_ROW+2):
                if purple_mask[j, i] == 255:
                    purple_pixels += 1
                    if purple_pixels > 1 and delta_t > 3:
                        last_purple = cur_time
                        return True
        return False
    def locate_wall(self, row_num, image):
        upper_wall = np.array([50, 70, 70])
        lower_wall = np.array([25, 45, 50])
        wall_mask = cv2.inRange(image, lower_wall, upper_wall)
        wall_pixel_counter = 0
        for i in range(COLUMNS_IN_IMAGE):
            if wall_mask[row_num, i] == 255:
                wall_pixel_counter += 1
            else:
                wall_pixel_counter = 0
            if wall_pixel_counter > 130:
                return i
        # cv2.imshow("wall", wall_mask)
        # cv2.waitKey(3)
        return -1

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
            if(colour ==255): #if Colour is close to white it is road
                road_pixel_counter += 1
            else: #if coulour is not white than reset counter
                road_pixel_counter = 0
            if road_pixel_counter > 12: #if greater than 10 consecutive columns are road, return pixel num
                if from_left:
                    return i
                else:
                    return COLUMNS_IN_IMAGE - i
        
        return -1 # value returned if there is no road on the screen

    def get_steering_val(self,speed=ROBOT_SPEED):
        """
        @brief modifies prev_steering_val, move
        uses steering value to find the track, then follow it, updating move each time it is called
        """
        global move
        if(self.steering_val == -1 and self.prev_steering_val == -1): #robot initialization - face road
            move.linear.x = 0
            move.angular.z = 0.5
        elif(self.prev_steering_val == -1) : #first time seeing road, begin driving forward
            self.prev_steering_val = self.steering_val
<<<<<<< HEAD
            move.linear.x = 0.2
            move.angular.z = 0
        elif(self.steering_val != -1): #if seeing road, set move command based on difference of road position from center
            move.linear.x = 0.2
            move.angular.z = -(self.steering_val-STEERING_CENTER)/150
=======
            move.linear.x = speed
            move.angular.z = 0
        elif(self.steering_val != -1): #if seeing road, set move command based on difference of road position from center
            move.linear.x = speed
            move.angular.z = -(self.steering_val-STEERING_CENTER)/125
>>>>>>> d5496e0f9f77218e0d4d88c17072e485ebea95de
            self.prev_steering_val = self.steering_val
        else:
            move.linear.x = 0.0 #if road is lost, rotate in direction road was last seen until road is found again
            move.angular.z = -(self.prev_steering_val-STEERING_CENTER)/125


    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and road is detected, steering val is set'''

        global counter, move, state_machine, seen_purple_lines, wall_seen
        cv_image = self.bridge.imgmsg_to_cv2(ros_data, 'bgr8')
        blur_image = cv2.GaussianBlur(cv_image, (3,3), 0)
        blur_image2 = cv2.GaussianBlur(blur_image, (7,7), 0)

        greyscale = cv2.cvtColor(blur_image2, cv2.COLOR_BGR2GRAY)

        # Define the lower and upper bounds for black (low brightness)
        lower_white = np.array([220])  # Lower bound
        upper_white = np.array([255])  # Upper bound 

        black_mask = cv2.inRange(greyscale, lower_white, upper_white)


        if (state_machine == NORMAL_DRIVE):
            line_position = self.locate_road(SCAN_ROW,black_mask)
            #cv2.circle(cv_image, (line_position, SCAN_ROW), 5, (0,0,255), -1)
            self.steering_val = line_position
            self.get_steering_val(speed=ROBOT_SPEED+.02)
            if counter == 300:
                state_machine = TRUCK_LOOP
                move.linear.x = 0
                move.angular.z = 1
                print("entering truck loop")

        elif state_machine == PEDESTRIAN:
            line_position = self.locate_road(SCAN_ROW,black_mask)
            state_machine = NORMAL_DRIVE
        elif state_machine == TRUCK_LOOP:
            line_position = min([self.__scan_row_for_road(SCAN_ROW,black_mask,True) + 350,1279])
        
            #cv2.circle(cv_image, (line_position, SCAN_ROW), 5, (0,0,255), -1)
            self.steering_val = line_position
            if line_position == -1:
                line_position = 1
            self.get_steering_val()
            if counter == 600:
                state_machine = NORMAL_DRIVE
                print("entering normal drive")
        elif state_machine == GRASS_ROAD:
            upper_line = np.array([180, 230, 230])
            lower_line = np.array([130, 170, 180])
            line_mask = cv2.inRange(blur_image2, lower_line, upper_line)
            #cv2.imshow("Mask2", line_mask)
            line_position = self.locate_road(GRASS_ROW,line_mask)
            line2 = self.locate_road(SCAN_ROW,line_mask)
            #line3 = self.locate_road(GRASS_2,black_mask)
            line_position = (line_position + line2 )/2
            #cv2.circle(cv_image, (line_position, GRASS_ROW), 5, (0,0,255), -1)
            self.steering_val = line_position
            self.get_steering_val(speed=ROBOT_SPEED+.03)

        elif state_machine == OFFROAD:
            if not wall_seen:
                move.linear.x = 0.4
                move.angular.z = 0.2
                line_position = 0
            wall = self.locate_wall(WALL_ROW, blur_image2)
            if wall != -1:
                print("wall seen")
                wall_seen = True
                move.linear.x = 0.9
                move.angular.z = -0.3
                line_position = 0
                #cv2.circle(cv_image, (line_position, WALL_ROW), 5, (0,0,255), -1)
            else:
                move.linear.x = 0.8
                move.angular.z = 0.3
                line_position = 0
        elif state_machine == MOUNTAIN:
            self.prev_steering_val = -1
            upper_line = np.array([180, 230, 230])
            lower_line = np.array([130, 170, 180])
            line_mask = cv2.inRange(blur_image2, lower_line, upper_line)
            line_position = self.locate_road(GRASS_ROW,line_mask)
            self.steering_val = line_position
            self.get_steering_val(speed=ROBOT_SPEED-0.1)
            


        else:
            line_position = self.locate_road(SCAN_ROW,black_mask)
            state_machine = NORMAL_DRIVE

        if self.check_for_purple(blur_image):
            seen_purple_lines += 1
            print("PURPLE")
            if seen_purple_lines == 1:
                state_machine = GRASS_ROAD
                print("entering grass road")
            if seen_purple_lines == 2:
                state_machine = OFFROAD
                print("entering offroad")
            if seen_purple_lines == 3:
                move.linear.x = 0
                move.angular.z = 0
                state_machine = MOUNTAIN
                print("entering mountain")
        
        self.publisher.publish(move)
        

        if line_position == -1:
            state = self.prev_state
        else:
            state = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            line_position = math.floor(line_position/COLUMNS_IN_IMAGE*len(state)-1)
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

        if counter >= 2000:
            rospy.sleep(3)
            move.linear.x = 0
            move.angular.z = 0
            self.publisher.publish(move)
            self.publisher2.publish('TeamName,password,-1,NA')
            rospy.sleep(1)
            os.kill(os.getpid(),9)

        counter += 1
        #print(self.steering_val)
        
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
