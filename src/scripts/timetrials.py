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
TRUCK_MASK_TOP = 360
TRUCK_MASK_LEFT = 300
TRUCK_MASK_RIGHT = 600
VERBOSE=False
NORMAL_DRIVE = 0
PEDESTRIAN = 1
TRUCK_LOOP = 2
OFFROAD = 3
MOUNTAIN_BASE = 4
GRASS_ROAD = 5 
TUNNEL_MOUTH = 6
TUNNEL_INSIDE = 7
CLIMBING_MOUNTAIN = 8
CLIMBING_MOUNTAIN2 = 9
CLIMBING_MOUNTAIN3 = 10
INTERSECTION = 11

TUNNEL_UPPER_HSV = np.array([13, 153, 222])
TUNNEL_LOWER_HSV = np.array([2, 122, 153])
TUNNEL_INSIDE_UPPER_HSV = np.array([14, 168, 100])
TUNNEL_INSIDE_LOWER_HSV = np.array([1, 27, 54])


MOUNTAIN_LOWER_HSV = np.array([16, 40, 156])
MOUNTAIN_UPPER_HSV = np.array([52, 101, 215])
MOUNTAIN2_LOWER_HSV = np.array([8, 39, 178])
MOUNTAIN2_UPPER_HSV = np.array([54, 100, 233])

CROSSWALK_RED_LOWER_BGR = np.array([0, 0, 250])
CROSSWALK_RED_UPPER_BGR = np.array([5, 5, 255])

PED_PANTS_UPPER_HSV = np.array([160,180,93])
PED_PANTS_LOWER_HSV = np.array([91,49,23])

TRUCK_MASK_UPPER_HSV = np.array([51, 16, 246])
TRUCK_MASK_LOWER_HSV = np.array([0, 0, 46])

move = Twist()
counter = 0
state_machine = NORMAL_DRIVE

ROBOT_SPEED = 0.4

seen_purple_lines = 0
wall_seen = False
road_located_after_yoda = False
frames_since_line = 0
last_purple = time.time()

class robot_driving:
    """Class that subsribes to image feed"""

    def __init__(self):
        '''Initialize ros subscriber'''
        self.steering_val = STEERING_CENTER
        self.prev_steering_val = STEERING_CENTER
        self.prev_state = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.bridge = CvBridge()
        self.tunnel_seen = False
        self.count_at_tunnel = 0
        self.tunnel_brightness = 0
        self.mountain2_brightness = 0
        self.signcount = 0
        self.pedestrian_initial_position = np.zeros((300, 1279))
        self.last_truck_position = np.zeros((BOTTOM_ROW_OF_IMAGE-200-TRUCK_MASK_TOP, 1279))
        self.ped_seen = False
        self.stopped_for_ped = False
        self.stopped_for_truck = False
        self.truck_seen = False   
        rospy.sleep(20)
        # subscribed Topic
        self.publisher2 = rospy.Publisher("/score_tracker",String, queue_size=1)
        self.signsub = rospy.Subscriber("sign_detected", String, self.signcounter, queue_size=1)
        self.subscriber = rospy.Subscriber("/R1/pi_camera/image_raw",
            Image, self.callback,  queue_size = 1)
        rospy.sleep(1)
        self.publisher2.publish('TeamName,password,0,NA')
        self.publisher = rospy.Publisher("/R1/cmd_vel",Twist, queue_size=1)
        '''initialize ros publisher'''
        global move
        # move.linear.x = 0.5
        #self.publisher.publish(move)


        if VERBOSE :
            print ("/rrbot/camera/image_raw")
    
    def signcounter(self, data):
        self.signcount +=1
        print("signcount: ", self.signcount) 


    def check_for_purple(self, image):
        cur_time = time.time()
        global last_purple
        delta_t = cur_time - last_purple
        lower_purple = np.array([200, 0, 200])
        upper_purple = np.array([255, 15, 255])
        purple_pixels = 0
        purple_mask = cv2.inRange(image, lower_purple, upper_purple)
        #cv2.imshow("purple", purple_mask)
        for i in range(COLUMNS_IN_IMAGE):
            for j in range(GRASS_ROW-2, GRASS_ROW+2):
                if purple_mask[j, i] == 255:
                    purple_pixels += 1
                    if purple_pixels > 1 and delta_t > 3:
                        last_purple = cur_time
                        return True
                    
                    #in to cm
                    #ft to cm
                    #kg to lbs
                    #L to US gal
                    # c to f
                    #W to HP
                    #Explaiin diff between slugs lbm lbf?
                    #vs force being derived in metric
                    #slug is the derived unit in imperial system
                    #1lb force = 1lb mass at 1g
                    #apply 1lb force to 1lb mass, it will accelerate at 1ft/s^2
                    #1lbf = 1 slug * 1ft/s^2

                    #BONUS
                    #wrench size?
                    #HP to to ft lb/s?

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

    def locate_road(self, row_num, mask, from_center = False):
        """@brief scans a row of image for the road, returns pixel num of some point on the road (counting up
        from left hand side of the screen)
        @param row_num the number of the row being scanned (top to bottom)
        @param mask the mask to be scanned
        @retval value of road pixel, measured from left of screen
        """
        if from_center:
            left_road_pixel = self.__scan_row_for_road(row_num,mask,True,True)
            right_road_pixel = self.__scan_row_for_road(row_num,mask,False,True)
        else:
            left_road_pixel = self.__scan_row_for_road(row_num,mask,True)
            right_road_pixel = self.__scan_row_for_road(row_num,mask,False)
        if left_road_pixel != -1 and right_road_pixel != -1:
            return math.floor((left_road_pixel + right_road_pixel)/2)
        return -1
    
    def __scan_row_for_road(self,row_num,mask, to_right = True, from_center = False):
        image = mask
        road_pixel_counter = 0
        if from_center:
            #start from the middle of the image and scan outwards
            for i in range(STEERING_CENTER):
                if to_right:
                    colour = image[row_num,STEERING_CENTER+i]
                else:
                    colour = image[row_num,STEERING_CENTER-i]
                if(colour ==255):
                    road_pixel_counter += 1
                else:
                    road_pixel_counter = 0
                if road_pixel_counter > 10:
                    if to_right:
                        return STEERING_CENTER+i
                    else:
                        return STEERING_CENTER-i

        for i in range(COLUMNS_IN_IMAGE): #loop through row, getting pixel val for every col
            if to_right:
                colour = image[row_num,i]
            else:
                colour = image[row_num,COLUMNS_IN_IMAGE - i]
            if(colour ==255): #if Colour is close to white it is road
                road_pixel_counter += 1
            else: #if coulour is not white than reset counter
                road_pixel_counter = 0
            if road_pixel_counter > 12: #if greater than 10 consecutive columns are road, return pixel num
                if to_right:
                    return i
                else:
                    return COLUMNS_IN_IMAGE - i
        
        return -1 # value returned if there is no road on the screen

    def get_steering_val(self,speed=ROBOT_SPEED, steering_sensitivity = 105):
        """
        @brief modifies prev_steering_val, move
        uses steering value to find the track, then follow it, updating move each time it is called
        """
        global move
        if(self.steering_val == -1 and self.prev_steering_val == -1): #robot initialization - face road
            move.linear.x = 0.2
            move.angular.z = 0.5
        elif(self.prev_steering_val == -1) : #first time seeing road, begin driving forward
            self.prev_steering_val = self.steering_val
            move.linear.x = speed
            move.angular.z = 0
        elif(self.steering_val != -1): #if seeing road, set move command based on difference of road position from center
            move.linear.x = speed
            move.angular.z = -(self.steering_val-STEERING_CENTER)/steering_sensitivity
            self.prev_steering_val = self.steering_val
        else:
            move.linear.x = 0.0 #if road is lost, rotate in direction road was last seen until road is found again
            move.angular.z = -(self.prev_steering_val-STEERING_CENTER)/steering_sensitivity


    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and road is detected, steering val is set'''

        global counter, move, state_machine, seen_purple_lines, wall_seen, frames_since_line
        cv_image = self.bridge.imgmsg_to_cv2(ros_data, 'bgr8')
        cv_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        blur_image = cv2.GaussianBlur(cv_image, (3,3), 0)
        blur_image2 = cv2.GaussianBlur(blur_image, (7,7), 0)

        greyscale = cv2.cvtColor(blur_image2, cv2.COLOR_BGR2GRAY)

        # Define the lower and upper bounds for black (low brightness)
        lower_white = np.array([220])  # Lower bound
        upper_white = np.array([255])  # Upper bound 

        black_mask = cv2.inRange(greyscale, lower_white, upper_white)


        if (state_machine == NORMAL_DRIVE):
            if not self.ped_seen:
                red_mask = cv2.inRange(blur_image2, CROSSWALK_RED_LOWER_BGR, CROSSWALK_RED_UPPER_BGR)
                red_line = self.locate_road(SCAN_ROW, red_mask)
                if red_line != -1:
                    move.linear.x = 0
                    move.angular.z = 0
                    state_machine = PEDESTRIAN
                    #the location of the pedestrian is the largest white area in the bottom 3/4 of the image
                    print("entering pedestrian")
                    return
            line_position = self.locate_road(SCAN_ROW,black_mask)
            #cv2.circle(cv_image, (line_position, SCAN_ROW), 5, (0,0,255), -1)
            self.steering_val = line_position
            self.get_steering_val(speed=ROBOT_SPEED+.12, steering_sensitivity=70)
            #if red line is detected, stop and wait for pedestrian to cross

            if self.signcount == 2 and counter > 350:
               move.linear.x = 0
               move.angular.z = 0.3
               self.publisher.publish(move)
               rospy.sleep(1)
               state_machine = INTERSECTION
               return

            
            if self.signcount == 3:

                state_machine = INTERSECTION
                move.linear.x = 0
                move.angular.z = 0
                print("entering intersection")

        elif state_machine == PEDESTRIAN:
            hsv_blur = cv2.GaussianBlur(cv_hsv, (3,3), 0)
            ped_mask = cv2.inRange(hsv_blur, PED_PANTS_LOWER_HSV, PED_PANTS_UPPER_HSV)
            ped_mask = ped_mask[BOTTOM_ROW_OF_IMAGE-500:BOTTOM_ROW_OF_IMAGE-200,0:COLUMNS_IN_IMAGE]/255
            move.linear.x = 0
            move.angular.z = 0
            self.publisher.publish(move)
            if not self.stopped_for_ped:
                move.linear.x = 0
                move.angular.z = 0
                self.publisher.publish(move)
                rospy.sleep(1)
                self.stopped_for_ped = True
                return
            if not self.ped_seen:
                hsv_blur = cv2.GaussianBlur(cv_hsv, (3,3), 0) 
                ped_mask = cv2.inRange(hsv_blur, PED_PANTS_LOWER_HSV, PED_PANTS_UPPER_HSV)
                ped_mask = ped_mask[BOTTOM_ROW_OF_IMAGE-500:BOTTOM_ROW_OF_IMAGE-200,0:COLUMNS_IN_IMAGE]/255
                self.pedestrian_initial_position = ped_mask
                self.ped_seen = True
                
                return
            line_position = self.locate_road(SCAN_ROW,black_mask)
                #wait for the pedestrian to move by checking if the white area has moved significantly
            difference = cv2.absdiff(ped_mask, self.pedestrian_initial_position)
            print(np.sum(abs(difference)))
            cv2.imshow("ped", ped_mask)
            cv2.waitKey(3)
            if np.sum(abs(difference)) > 645:
                state_machine = NORMAL_DRIVE
                print("exiting pedestrian")
            return
        
        elif state_machine == INTERSECTION:
            hsv_blur = cv2.GaussianBlur(cv_hsv, (3,3), 0)
            truck_mask = cv2.inRange(hsv_blur, TRUCK_MASK_LOWER_HSV, TRUCK_MASK_UPPER_HSV)
            truck_mask = truck_mask[TRUCK_MASK_TOP:BOTTOM_ROW_OF_IMAGE-200,TRUCK_MASK_LEFT:TRUCK_MASK_RIGHT]/255
            move.linear.x = 0
            move.angular.z = 0
            self.publisher.publish(move)
            
            if not self.truck_seen:
                self.last_truck_position= truck_mask 
                self.truck_seen = True
                return
            line_position = self.locate_road(SCAN_ROW,black_mask)
                #wait for the pedestrian to move by checking if the white area has moved significantly
            difference = cv2.absdiff(truck_mask, self.last_truck_position)
            print(np.sum(abs(difference)))
            cv2.imshow("truck", truck_mask)
            cv2.waitKey(3)
            if np.sum(abs(difference)) < 100:
                state_machine = TRUCK_LOOP
                print("exiting intersection")
                return
            rospy.sleep(0.4)
            self.last_truck_position = truck_mask
            return

        elif state_machine == TRUCK_LOOP:
            line_position = min([self.__scan_row_for_road(SCAN_ROW,black_mask,True) + 350,1279])
            
        
            #cv2.circle(cv_image, (line_position, SCAN_ROW), 5, (0,0,255), -1)
            self.steering_val = line_position
            if line_position == -1:
                line_position = 1
            self.get_steering_val()
            
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
            if(self.signcount == 5):
                self.get_steering_val(speed=ROBOT_SPEED, steering_sensitivity=110)
            else:  
                self.get_steering_val(speed=ROBOT_SPEED-0.13, steering_sensitivity=85)

        elif state_machine == OFFROAD:
            if not wall_seen:
                move.linear.x = 0.4
                move.angular.z = 0.2
                line_position = 0
            wall = self.locate_wall(WALL_ROW, blur_image2)
            if wall != -1:
                print("wall seen")
                wall_seen = True
                move.linear.x = 1
                move.angular.z = -0.25
                line_position = 0
                #cv2.circle(cv_image, (line_position, WALL_ROW), 5, (0,0,255), -1)
            else:
                move.linear.x = 1.05
                move.angular.z = 0.42
                line_position = 0
        elif state_machine == MOUNTAIN_BASE:
            if(counter - self.count_at_tunnel < 30):
                frames_since_line += 1
                move.linear.x = 0.14
                move.angular.z = 0.35
                line_position = -1
            else:
                line_position = -1
                self.count_at_tunnel = counter
                state_machine = TUNNEL_MOUTH
                print("entering tunnel")
        elif state_machine == TUNNEL_MOUTH:
            hsv_blur = cv2.GaussianBlur(cv_hsv, (3,3), 0)
            tunnel_mask = cv2.inRange(hsv_blur, TUNNEL_LOWER_HSV, TUNNEL_UPPER_HSV)
            dark_tunnel_mask = cv2.inRange(hsv_blur, TUNNEL_INSIDE_LOWER_HSV, TUNNEL_INSIDE_UPPER_HSV)
            line_position = self.locate_road(SCAN_ROW-180, tunnel_mask, from_center=False)
            self.steering_val = min([line_position+25, 1279])
            line_position_inside = self.locate_road(SCAN_ROW-100, dark_tunnel_mask, from_center=False)
            if line_position_inside != -1 and counter - self.count_at_tunnel > 13:
                state_machine = TUNNEL_INSIDE
                frames_since_line = 0
                print("inside tunnel")
            self.get_steering_val(speed=ROBOT_SPEED-0.3, steering_sensitivity=120)
            
            cv2.imshow("tunnel", tunnel_mask)
        elif state_machine == TUNNEL_INSIDE:
            hsv_blur = cv2.GaussianBlur(cv_hsv, (3,3), 0)
            tunnel_mask = cv2.inRange(hsv_blur, TUNNEL_INSIDE_LOWER_HSV, TUNNEL_INSIDE_UPPER_HSV)
            line_position = self.locate_road(325, tunnel_mask, from_center=True)
            self.steering_val = line_position
            if line_position == -1:
                state_machine = CLIMBING_MOUNTAIN
                self.tunnel_brightness = np.mean(hsv_blur[BOTTOM_ROW_OF_IMAGE-100:BOTTOM_ROW_OF_IMAGE,0:COLUMNS_IN_IMAGE])
                print("climing the mountain")
            self.get_steering_val(speed=ROBOT_SPEED-0.2, steering_sensitivity=125)
            cv2.imshow("tunnel", tunnel_mask)
        elif state_machine == CLIMBING_MOUNTAIN:
            hsv_blur = cv2.GaussianBlur(cv_hsv, (3,3), 0)
            mountain_mask = cv2.inRange(hsv_blur, MOUNTAIN_LOWER_HSV, MOUNTAIN_UPPER_HSV)
            line_position = self.locate_road(SCAN_ROW, mountain_mask)
            line2 = self.locate_road(GRASS_ROW, mountain_mask)
            line_position = (line_position + line2 )/2
            self.steering_val = line_position
            self.get_steering_val(speed=ROBOT_SPEED)
            #change back to grass road when brightness of bottom quarter of image increases
            brightness = np.mean(cv_hsv[BOTTOM_ROW_OF_IMAGE-100:BOTTOM_ROW_OF_IMAGE,0:COLUMNS_IN_IMAGE])
            if brightness - self.tunnel_brightness > 15:
                self.mountain2_brightness = brightness    
                state_machine = CLIMBING_MOUNTAIN2
                print("entering mountain2")
        elif state_machine == CLIMBING_MOUNTAIN2:
            hsv_blur = cv2.GaussianBlur(cv_hsv, (3,3), 0)
            mountain_mask = cv2.inRange(hsv_blur, MOUNTAIN2_LOWER_HSV, MOUNTAIN2_UPPER_HSV)
            line_position = self.locate_road(SCAN_ROW, mountain_mask)
            line2 = self.locate_road(GRASS_ROW, mountain_mask)
            line_position = (line_position + line2 )/2
            if line2 == -1:
                self.steering_val = STEERING_CENTER
            else:
                self.steering_val = line_position
            self.get_steering_val(speed=ROBOT_SPEED-0.1)
            brightness = np.mean(cv_hsv[BOTTOM_ROW_OF_IMAGE-100:BOTTOM_ROW_OF_IMAGE,0:COLUMNS_IN_IMAGE])
            cv2.imshow("mountain", mountain_mask)



        
            
            


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
                state_machine = MOUNTAIN_BASE
                self.count_at_tunnel = counter
                self.prev_steering_val = -1
                print("entering mountain")

        try: 
            if(state_machine == PEDESTRIAN):
                move.linear.x = 0
                move.angular.z = 0         
            self.publisher.publish(move)
        except:
            print("failed to publish")
        

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

        if counter >= 3500:
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
