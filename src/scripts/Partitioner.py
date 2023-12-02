import cv2
import numpy as np
import os
import random

# Define the path to the image within this environment
image_path = '/home/fizzer/ros_ws/src/controller_pkg/src/scripts/CRIMEBIRRADIATE.jpg'

# Define the directory to save individual characters
save_dir = '/home/fizzer/ros_ws/src/controller_pkg/src/scripts/IndividualCharacters/'


image = cv2.imread(image_path)

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

if not os.path.exists(save_dir):
    os.makedirs(save_dir)

# List to store saved image file paths for review
saved_images = []




# Extract filename without extension and convert it to a list of characters
filename_no_extension = os.path.splitext(os.path.basename(image_path))[0]
characters = list(filename_no_extension)

# Counter for characters
char_counter = 0

# Extract and save individual characters
for i, cnt in enumerate(sorted_contours):
    x, y, w, h = cv2.boundingRect(cnt)

    if x < boundary_x and y < height // 2:
        continue

    # Too get rid of extraneous images
    if w < 16 or h < 16:
         continue

    # Extract the character
    char_image = image[y:y+h, x:x+w]

    # Check if there are enough characters left in the list
    if char_counter < len(characters):

        # If the width is at least 1.4 times the height and less than twice, split it into two
        if 1.4 * h <= w < 2.0 * h:
            char_name_Left = characters[char_counter]
            char_counter+=1
            order_indexLR = char_counter
            char_name_Right = characters[char_counter]

            mid = w // 2
            char_image_left = char_image[:, :mid]
            char_image_right = char_image[:, mid:]

            # Generate random numbers for naming
            random_number_left = random.randint(10, 99)
            random_number_right = random.randint(10, 99)

            # Save the left part of the character image
            filename_left = os.path.join(save_dir, f'{char_name_Left}_{order_indexLR-1}_{random_number_left}.png')
            cv2.imwrite(filename_left, char_image_left)
            saved_images.append(filename_left)

            # Save right part
            filename_right = os.path.join(save_dir, f'{char_name_Right}_{order_indexLR}_{random_number_right}.png')
            cv2.imwrite(filename_right, char_image_right)
            saved_images.append(filename_right)


            char_counter += 1  # Increment character counter after saving both parts

        # If the width is at least twice the height, split it into three
        elif w >= 2.0 * h:
            char_name_L = characters[char_counter]
            char_counter+=1
            char_name_M = characters[char_counter]
            char_counter+=1
            order_indexLMR = char_counter # To get characters in order
            char_name_R = characters[char_counter]

            part_width = w // 3
            char_image_left = char_image[:, :part_width]
            char_image_middle = char_image[:, part_width:2*part_width]
            char_image_right = char_image[:, 2*part_width:]


            # Generate random numbers for each filename
            random_number_left = random.randint(10, 99)
            random_number_middle = random.randint(10, 99)
            random_number_right = random.randint(10, 99)

            # Save left part
            filename_left = os.path.join(save_dir, f'{char_name_L}_{order_indexLMR-2}_{random_number_left}.png')
            cv2.imwrite(filename_left, char_image_left)
            saved_images.append(filename_left)

            # Save middle part
            filename_middle = os.path.join(save_dir, f'{char_name_M}_{order_indexLMR-1}_{random_number_middle}.png')
            cv2.imwrite(filename_middle, char_image_middle)
            saved_images.append(filename_middle)

            # Save right part
            filename_right = os.path.join(save_dir, f'{char_name_R}_{order_indexLMR}_{random_number_right}.png')
            cv2.imwrite(filename_right, char_image_right)
            saved_images.append(filename_right)

            char_counter += 1  # Increment character counter after saving all parts

        else:
            # Save the character image
            char_name = characters[char_counter]

            random_num = random.randint(10, 99)

            filename = os.path.join(save_dir, f'{char_name}_{char_counter}_{random_num}.png')
            cv2.imwrite(filename, char_image)
            saved_images.append(filename)

            char_counter += 1  # Increment character counter after saving the character


