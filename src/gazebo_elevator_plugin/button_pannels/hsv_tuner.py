import cv2
import numpy as np
import os

def nothing(x):
    pass

# Load your elevator panel image
# Replace with the path to one of your pictures
image_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'floor02_in_modified.jpeg')
image = cv2.imread(image_path)

if image is None:
    print(f"Error: Could not load image at {image_path}")
    exit()

# Optionally resize if the image is massive (e.g., from a phone camera)
# image = cv2.resize(image, (640, 480))

# Create a window
cv2.namedWindow('HSV Tuner', cv2.WINDOW_NORMAL)
cv2.resizeWindow('HSV Tuner', 600, 300)

# Create trackbars for color change
# Hue is from 0-179 for OpenCV
cv2.createTrackbar('H Min', 'HSV Tuner', 0, 179, nothing)
cv2.createTrackbar('S Min', 'HSV Tuner', 0, 255, nothing)
cv2.createTrackbar('V Min', 'HSV Tuner', 0, 255, nothing)
cv2.createTrackbar('H Max', 'HSV Tuner', 179, 179, nothing)
cv2.createTrackbar('S Max', 'HSV Tuner', 255, 255, nothing)
cv2.createTrackbar('V Max', 'HSV Tuner', 255, 255, nothing)

# Set default values for a rough "orange" starting point
cv2.setTrackbarPos('H Min', 'HSV Tuner', 5)
cv2.setTrackbarPos('S Min', 'HSV Tuner', 100)
cv2.setTrackbarPos('V Min', 'HSV Tuner', 150)
cv2.setTrackbarPos('H Max', 'HSV Tuner', 25)

while True:
    # Convert image to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Get current positions of all trackbars
    hMin = cv2.getTrackbarPos('H Min', 'HSV Tuner')
    sMin = cv2.getTrackbarPos('S Min', 'HSV Tuner')
    vMin = cv2.getTrackbarPos('V Min', 'HSV Tuner')
    hMax = cv2.getTrackbarPos('H Max', 'HSV Tuner')
    sMax = cv2.getTrackbarPos('S Max', 'HSV Tuner')
    vMax = cv2.getTrackbarPos('V Max', 'HSV Tuner')

    # Set minimum and maximum HSV values to display
    lower = np.array([hMin, sMin, vMin])
    upper = np.array([hMax, sMax, vMax])

    # Create a mask and apply it to the image
    mask = cv2.inRange(hsv, lower, upper)
    result = cv2.bitwise_and(image, image, mask=mask)

    # Display the resulting frames
    cv2.imshow('Original Image', image)
    cv2.imshow('Mask (White = Detected)', mask)
    cv2.imshow('Color Filtered Result', result)

    # Press 'q' or ESC to exit
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q') or key == 27:
        print(f"Final HSV Lower Bound: [{hMin}, {sMin}, {vMin}]")
        print(f"Final HSV Upper Bound: [{hMax}, {sMax}, {vMax}]")
        break

cv2.destroyAllWindows()