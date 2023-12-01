import cv2
import numpy as np
import os

# Define the path to the image within this environment
image_path = '/home/fizzer/ros_ws/src/controller_pkg/src/scripts/sign_6.jpg'

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

    # If the width is at least 1.4 times the height and less than twice, split it into two
    if 1.4 * h <= w < 2.0 * h:
        mid = w // 2
        char_image_left = char_image[:, :mid]
        char_image_right = char_image[:, mid:]

        # Save the left part of the character image
        filename_left = os.path.join(save_dir, f'char_{i+1}_left.png')
        cv2.imwrite(filename_left, char_image_left)
        saved_images.append(filename_left)

        # Save the right part of the character image
        filename_right = os.path.join(save_dir, f'char_{i+1}_right.png')
        cv2.imwrite(filename_right, char_image_right)
        saved_images.append(filename_right)

    # If the width is at least twice the height, split it into three
    elif w >= 2.0 * h:
        part_width = w // 3
        char_image_left = char_image[:, :part_width]
        char_image_middle = char_image[:, part_width:2*part_width]
        char_image_right = char_image[:, 2*part_width:]

        # Save each part of the character image
        filename_left = os.path.join(save_dir, f'char_{i+1}_left.png')
        cv2.imwrite(filename_left, char_image_left)
        saved_images.append(filename_left)

        filename_middle = os.path.join(save_dir, f'char_{i+1}_middle.png')
        cv2.imwrite(filename_middle, char_image_middle)
        saved_images.append(filename_middle)

        filename_right = os.path.join(save_dir, f'char_{i+1}_right.png')
        cv2.imwrite(filename_right, char_image_right)
        saved_images.append(filename_right)

    else:
        # Save the character image
        filename = os.path.join(save_dir, f'char_{i+1}.png')
        cv2.imwrite(filename, char_image)
        saved_images.append(filename)

