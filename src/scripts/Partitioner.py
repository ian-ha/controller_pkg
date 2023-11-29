import cv2
import numpy as np
import os

# Define the path to the image within this environment
image_path = '/home/fizzer/ros_ws/src/controller_pkg/src/scripts/TestImage.jpg'

# Define the directory to save individual characters
save_dir = '/home/fizzer/ros_ws/src/controller_pkg/src/scripts/IndividualCharacters/'

# Create the directory if it does not exist
if not os.path.exists(save_dir):
    os.makedirs(save_dir)

# Check if the image file exists
if not os.path.exists(image_path):
    print(f"Error: The file at {image_path} does not exist.")
else:
    # Load the image
    image = cv2.imread(image_path)

    # Check if the image was loaded successfully
    if image is None:
        print(f"Error: The image at {image_path} could not be loaded.")
    else:
        # Convert to grayscale and apply Gaussian blur
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)

        # Adaptive thresholding on the blurred image
        thresh = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                       cv2.THRESH_BINARY_INV, 11, 2)

        # Morphological operations to separate characters that are close together
        kernel = np.ones((2, 2), np.uint8)
        thresh = cv2.dilate(thresh, kernel, iterations=1)
        thresh = cv2.erode(thresh, kernel, iterations=1)

        # Find contours
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Function to sort contours from left-to-right
        def sort_contours(cnts):
            boundingBoxes = [cv2.boundingRect(c) for c in cnts]
            cnts, boundingBoxes = zip(*sorted(zip(cnts, boundingBoxes), key=lambda b:b[1][0], reverse=False))
            return cnts

        sorted_contours = sort_contours(contours)

        # Get the width and height of the image
        height, width = image.shape[:2]

        # Calculate the boundary for the top left quarter
        boundary_x = width // 3
        boundary_y = height // 2

        # Extract and save individual characters
        for i, cnt in enumerate(sorted_contours):
            x, y, w, h = cv2.boundingRect(cnt)

            # Skip characters in the top left quarter of the image
            if x < boundary_x and y < boundary_y:
                continue

            # Make sure contour area is large enough to be a character and not too large
            if 50 < cv2.contourArea(cnt) < 1000 and h > 10 and w > 5:
                # Aspect ratio check for non-I/I-like characters
                aspect_ratio = float(w) / h
                if aspect_ratio < 1.5:
                    # Increase the bounding box size slightly to include the whole character
                    margin = 5
                    x, y, w, h = x - margin, y - margin, w + 2 * margin, h + 2 * margin
                    # Extract the character
                    char_image = image[y:y+h, x:x+w]
                    # Make sure the character image is not empty
                    if char_image.size > 0:
                        # Save the character image
                        filename = os.path.join(save_dir, f'char_{i}.jpg')
                        cv2.imwrite(filename, char_image)
                        print(f"Saved: {filename}")
                    else:
                        print(f"Warning: The character image for contour {i} is empty and will not be saved.")

        print("Characters extracted and saved.")
