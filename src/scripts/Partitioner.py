#! /usr/bin/env python3
import cv2
import numpy as np
import os
import random
import PIL.Image as Image
import tensorflow as tf
from tensorflow import keras
import ros
import rospy
from std_msgs.msg import String
import sys
import time

# Scale images
target_width = 42
target_height = 60

# HSV values
lower_hsv = np.array([80, 114, 60])
upper_hsv = np.array([170, 250, 230])
ONEHOT_INDEX = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"

# Define the path to the image within this environment
image_path = '/home/fizzer/ros_ws/src/controller_pkg/src/plateImages/'


MODEL_PATH = r'/home/fizzer/ros_ws/src/controller_pkg/src/models/big_data.keras'
PARTITIONED_IMG_PATH = r'/home/fizzer/ros_ws/src/controller_pkg/src/IndividualCharacters/'

class clueGuesser:
    def __init__(self):
        for filename in os.listdir(image_path):
            os.remove(os.path.join(image_path, filename))
        self.model = keras.models.load_model(MODEL_PATH, compile=False)
        self.model.compile(optimizer='adam', loss='categorical_crossentropy', metrics=['accuracy'])
        self.publisher = rospy.Publisher("/score_tracker",String, queue_size=2)
        self.clue_subscriber = rospy.Subscriber("sign_ready", String, self.callback, queue_size=1)
        self.clue_count = 0
        self.entries = {'SIZE': ["TWO", "314", "DOZEN", "RAYO10", "COUNTLESS", "LEGIONS",
                    "TRIPLETS"],
           'VICTIM': ["PARROTS", "ROBOTS", "BACTERIA", "JEDIS", "ALIENS",
                      "CITIZENS", "PHYSICISTS", "FRODO", "DINOSAURS", "BUNNIES",
                      "BED BUGS", "ANTS"],
           'CRIME': ["STEAL", "TRESPASS", "LIE TO", "DESTROY", "PUNCH", "BITE",
                     "TRANSMOGRIFY", "TELEPORT", "ACCELERATE", "IRRADIATE",
                     "CURSE", "HEADBUT", "DEFRAUD", "DECELERATE", "TICKLE"],
           'TIME': ["NOON", "MIDNIGHT", "DAWN", "DUSK", "JURASIC", "TWILIGHT",
                    "D DAY", "Q DAY", "2023", "WINTER", "SUMMER", "SPRING",
                    "AUTUMN"],
           'PLACE': ["HOSPITAL", "MALL", "FOREST", "MOON", "CLASS", "BEACH",
                     "JUNGLE", "BASEMENT", "THE HOOD", "SEWERS", "CAVE",
                     "BENU", "MARS"],
           'MOTIVE': ["GLUTTONY", "CURIOSITY", "IGNORANCE", "FEAR", "PRIDE",
                      "LOVE", "REVENGE", "PASSION", "BOREDOM", "THRILL",
                      "GREED", "FAME", "ACCIDENT", "HATE", "SELF DEFENSE"],
           'WEAPON': ["STICK", "ROCKET", "ANTIMATTER", "NEUTRINOS", "SHURIKEN",
                      "PENCIL", "PLASMA", "WATER", "FIRE", "POTATO GUN",
                      "ROPE", "ELECTRON", "HIGH VOLTAGE", "POLONIUM"],
           'BANDIT': ["EINSTEIN", "PIKACHU", "SHREK", "LUIGI", "BARBIE",
                      "BATMAN", "CAESAR", "SAURON", "THANOS", "GOKU",
                      "CAO CAO", "THE DEVIL", "GODZILA", "TEMUJIN",
                      "HANNIBAL"]
           }
        



    def get_char_for_net(self,contour, num_output_chars, width):
        preds = ""
        split = width // num_output_chars
        for i in range(num_output_chars):
            char_image = contour[:, split*i:split*(i+1)]
            if char_image is None:
                print("char_image is None")
                continue
            char_image = cv2.resize(char_image, (target_width, target_height))
            # Generate random numbers for naming
            hsv_image = cv2.cvtColor(char_image, cv2.COLOR_BGR2HSV)
            binary_mask = cv2.inRange(hsv_image, lower_hsv, upper_hsv)
            im_pil = Image.fromarray(binary_mask)
            im_pil = np.expand_dims(im_pil, axis=0)
            onehot = self.model.predict(im_pil)
            onehot = np.argmax(onehot)
            preds += ONEHOT_INDEX[onehot]
        return preds
    
    def callback(self, ros_data):
            rospy.sleep(1)
            for filename in os.listdir(image_path):
                time.sleep(1)
                if not filename.startswith('.'):
                    image = cv2.imread(os.path.join(image_path, filename))
                    sign_prediction = ""


                    if image is None:
                        print("char_image is None")
                        continue
                 

                    # Convert to grayscale and apply binary thresholding
                    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                    _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

                    # Morphological operations to separate characters that are close together
                    kernel = np.ones((2, 2), np.uint8)
                    morphed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

                    # Find contours
                    contours, _ = cv2.findContours(morphed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                    # Get the width and height of the image
                    height, width = image.shape[:2]

                    boundary_x = width // 3

                    # Function to calculate the row number based on y coordinate
                    def calculate_row(y, height):
                        return 0 if y < height / 2 else 1

                    # Sort the contours by row and x coordinate
                    sorted_contours = sorted(contours, key=lambda cnt: (calculate_row(cv2.boundingRect(cnt)[1], height), cv2.boundingRect(cnt)[0]))


                    # Extract and save individual characters
                    for i, cnt in enumerate(sorted_contours):
                        x, y, w, h = cv2.boundingRect(cnt)

                        if x < boundary_x and y < height // 2:
                            continue

                        # Too get rid of extraneous images
                        if w < 16 or h < 16:
                            continue

                        # Extract the character and resize image
                        char_image = image[y:y+h, x:x+w]


                        # If the width is at least 1.4 times the height and less than twice, split it into two
                        if 1.3 * h <= w < 2.0 * h:
                            sign_prediction += self.get_char_for_net(char_image,2,w)


                        # If the width is at least twice the height, split it into three
                        elif w >= 2.0 * h:
                            sign_prediction += self.get_char_for_net(char_image,3,w)


                        else:
                            # Save the character image
                            sign_prediction += self.get_char_for_net(char_image, 1,w)

                    print("Clue {}: {}".format(self.clue_count, sign_prediction))
                    #check if the beginning of the sign prediction is a key in entries
                    self.clue_count += 1
                    guess = ""
                    for key in self.entries:
                        if key in sign_prediction:
                            to_remove = len(key)
                            sign_prediction = sign_prediction[to_remove:]
                            guess = sign_prediction[:1] + ' ' + sign_prediction[1:]
                            self.publisher.publish('TeamName,password,'+str(self.clue_count)+','+guess)
                            break
                    if guess == "":
                        self.publisher.publish('TeamName,password,'+str(self.clue_count)+','+sign_prediction)
                    if self.clue_count == 8:
                        self.publisher.publish('TeamName,password,-1,woooo')
                    os.remove(os.path.join(image_path, filename))

def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('clue_guessing', anonymous=True)
    ic = clueGuesser()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

