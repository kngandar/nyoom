# Import necessary packages
from collections import deque
import numpy as np
import argparse
import imutils
import cv2

### CONSTANTS ###
# Define HSV range of colors (lower, upper)
green = [(29, 86, 6), (64, 255, 255)]
yellow = [(10, 120, 120), (50, 255, 255)]
red = [(145, 110, 120), (195, 255, 255)]

fontColor = (255, 165, 0)        # orange
outlineColor = (0, 255, 255)     # yellow

### DATA TRACKING ###
# Maybe (??)

### FUNCTIONS ###
def detectBalls(frame,hsv,color,numBalls):
    # Construct a mask for the color "green", then perform a series of
    # dilations and erosions to remove any small blobs left in the mask
    lower = color[0]
    upper = color[1]
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Find contours in the mask and initialize the current (x,y) center
    # of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    ballFound = []

    for i in range(1, numBalls):
        # Only proceed if at least one contour was found
        if len(cnts) > 0:
            cMax = max(cnts, key=cv2.contourArea)
            ((x,y), rad) = cv2.minEnclosingCircle(cMax)

            # If radius is greater than 10px, consider as ball
            if rad > 10:
                cv2.circle(frame, (int(x), int(y)), int(rad), outlineColor, 2)
                cv2.putText(frame, "r: " + str(int(rad), (int(x-rad),int(y-rad)),cv2.FONT_HERSHEY_SIMPLEX, 0.6, fontColor, 2)
                cv2.putText(frame, "x,y: " + str(int(x)) + ", " +  str(int(y)) , (int(x-rad),int(y-rad-rad)),cv2.FONT_HERSHEY_SIMPLEX, 0.6, fontColor, 2)
                ballPosition = [x,y,rad]
                ballFound.append(ballPosition)

            else
                print('All balls detected')
                break

            # Remove max contour from list
            cnts = [e for e in cnts if e not in (c)]

        # Returns positions of ball detected in current frame
        return ballFound

def main():
    # Open webcam object
    camera = cv2.VideoCapture(0)

    # Keep looping
    while True:
        # Grab the current frame
        (grabbed, frame) = camera.read()

        # If can't receive frame quit
        if not grabbed:
            break

        # Resize the frame, blur it, and convert it to the HSV color space
        frame = imutils.resize(frame, width=600)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        numBalls = 3

        ballFound = detectBall(frame,hsv,green,numBalls)
        if len(ballFound) > 0
            print('Number of balls detected: ' + str(len(ballFound)) )

        # Show the frame to our screen
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF

        # If the 'q' key is pressed, stop the loop
        if key == ord("q"):
            break
        elif key == ord("g"):
            print 'Set green thresh'
            color = green
        elif key == ord("y"):
            print 'Set yellow thresh'
            color = yellow
        elif key == ord("r"):
            print 'Set red thresh'
            color = red

    # Cleanup the camera and close any open windows
    camera.release()
    cv2.destroyAllWindows()



if __name__ == "__main__":
    main()