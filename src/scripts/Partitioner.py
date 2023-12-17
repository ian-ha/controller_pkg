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
        #self.number_to_key = {1: 'SIZE', 2: 'VICTIM', 3: 'CRIME', 4: 'TIME', 5: 'PLACE', 6: 'MOTIVE', 7: 'WEAPON', 8: 'BANDIT'}
        self.entries = {'SIZE': ["100", "10 GOOGLES", "314", "A PAIR", "BAKER DOZEN",
                    "COUNTLESS", "DOZEN", "FEW", "FIVE", "HALF DOZEN",
                    "LEGIONS", "MANY", "QUINTUPLETS", "RAYO10", "SINGLE",
                    "THREE", "TRIPLETS", "TWO", "UNCOUNTABLE", "ZEPTILLION"],
           'VICTIM': ["ALIENS", "ANTS", "BACTERIA", "BED BUGS", "BUNNIES",
                      "CITIZENS", "DINOSAURS", "FRODOS", "JEDIS", "KANGAROO",
                      "KOALAS", "PANDAS", "PARROTS", "PHYSICISTS", "QUOKKAS",
                      "ROBOTS", "RABBITS", "TOURISTS", "ZOMBIES"],
           'CRIME': ["ACCELERATE", "BITE", "CURSE", "DECELERATE", "DEFRAUD",
                     "DESTROY", "HEADBUT", "IRRADIATE", "LIE TO", "POKE",
                     "PUNCH", "PUSH", "SCARE", "STEAL", "STRIKE", "SWEAR",
                     "TELEPORT", "THINKING", "TICKLE", "TRANSMOGRIFY",
                     "TRESPASS"],
           'TIME': ["2023", "AUTUMN", "DAWN", "D DAY", "DUSK", "EONS AGO",
                    "JURASIC", "MIDNIGHT", "NOON", "Q DAY", "SPRING",
                    "SUMMER", "TOMORROW", "TWILIGHT", "WINTER", "YESTERDAY"],
           'PLACE': ["AMAZON", "ARCTIC", "BASEMENT", "BEACH", "BENU", "CAVE",
                     "CLASS", "EVEREST", "EXIT 8", "FIELD", "FOREST",
                     "HOSPITAL", "HOTEL", "JUNGLE", "MADAGASCAR", "MALL",
                     "MARS", "MINE", "MOON", "SEWERS", "SWITZERLAND",
                     "THE HOOD", "UNDERGROUND", "VILLAGE"],
           'MOTIVE': ["ACCIDENT", "BOREDOM", "CURIOSITY", "FAME", "FEAR",
                      "FOOLISHNESS", "GLAMOUR", "GLUTTONY", "GREED", "HATE",
                      "HASTE", "IGNORANCE", "IMPULSE", "LOVE", "LOATHING",
                      "PASSION", "PRIDE", "RAGE", "REVENGE", "REVOLT",
                      "SELF DEFENSE", "THRILL", "ZEALOUSNESS"],
           'WEAPON': ["ANTIMATTER", "BALOON", "CHEESE", "ELECTRON", "FIRE",
                      "FLASHLIGHT", "HIGH VOLTAGE", "HOLY GRENADE", "ICYCLE",
                      "KRYPTONITE", "NEUTRINOS", "PENCIL", "PLASMA",
                      "POLONIUM", "POSITRON", "POTATO GUN", "ROCKET", "ROPE",
                      "SHURIKEN", "SPONGE", "STICK", "TAMAGOCHI", "WATER",
                      "WRENCH"],
           'BANDIT': ["BARBIE", "BATMAN", "CAESAR", "CAO CAO", "EINSTEIN",
                      "GODZILA", "GOKU", "HANNIBAL", "L", "LENIN", "LUCIFER",
                      "LUIGI", "PIKACHU", "SATOSHI", "SHREK", "SAURON",
                      "THANOS", "TEMUJIN", "THE DEVIL", "ZELOS"]
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

                        if y < height // 2:
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

                    if self.clue_count != 5:
                        modified_sign_prediction = sign_prediction.replace("8", "B")
                        guess = modified_sign_prediction
                        
                    guess = sign_prediction
                    #self.publisher.publish('TeamName,password,' + str(self.clue_count) + ',' + guess)

                    number_to_key = {1: 'SIZE', 2: 'VICTIM', 3: 'CRIME', 4: 'TIME', 5: 'PLACE', 6: 'MOTIVE', 7: 'WEAPON', 8: 'BANDIT'}

                    # Reverse the mapping to easily find the integer key for a category
                    key_to_number = {v: k for k, v in number_to_key.items()}

                    for key in self.entries:
                        # Remove spaces from each element in the current category's list
                        entries_without_spaces = [entry.replace(" ", "") for entry in self.entries[key]]

                        # Check if guess is in the modified list (ignoring spaces)
                        if guess in entries_without_spaces:
                            # Get the integer key for the category
                            category_number = key_to_number[key]

                            # Publish the message with the integer key and guess
                            self.publisher.publish('TeamName,password,' + str(category_number) + ',' + guess)
                            break

                        
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

