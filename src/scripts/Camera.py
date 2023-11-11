#!/usr/bin/env python3

import rospy
import cv2
import sys
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class line_follower:

  def __init__(self):
    self.image_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/rrbot/camera1/image_raw", Image, self.callback)

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
    dark_pixel_indices_2 = np.where(brightness_values_2 < 188)[0]
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

    
    move = Twist()

    if(x_mid_of_frame-middle_spot>0):
        move.linear.x = 0.1
        move.angular.z = 0.5
    else:
        move.linear.x = 0.1
        move.angular.z = -0.5
    try:
      self.image_pub.publish(move)
    except CvBridgeError as e:
      print(e)



def main(args):
  lf = line_follower()
  rospy.init_node('line_follower', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)


