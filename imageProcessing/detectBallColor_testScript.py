# Import necessary packages
from collections import deque
import numpy as np
import argparse
import imutils
import cv2
import serial

# Define state
FIND_BALL = 1
FOUND_BALL = 2
SENT_CMD = 3

# Define HSV range of colors
green = [(29, 86, 6) , \
         (64, 255, 255)]
yellow = [(10, 120, 120), \
       (50, 255, 255)]
red = [(145, 110, 120), \
       (195, 255, 255)]

# Setup connection with Arduino
ser = serial.Serial('/dev/ttyACM0', 9600)

# Detect ball for specified hsv
def detectBall(frame,hsv,color):
    # Construct a mask for the color, then perform a series of
    # dilations and erosions to remove any small blobs left in the mask
    mask = cv2.inRange(hsv, color[0], color[1])
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
	
    # Find contours in the mask and initialize the current (x,y) center
    # of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None
	
    # Only proceed if at least one contour was found
    if len(cnts) > 0:
        # Find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and centroid
        c = max(cnts, key=cv2.contourArea)
        ((x,y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
		
	# Only proceed if the radius meets a minimum size
        if radius > 38:		
            # Draw the circle and centroid on the frame
            # then update the list of tracked points
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            return True
	else:
            return False
    else:
        return False

def main():
    # Create webcam object
    camera = cv2.VideoCapture(0)

    # Initialization
    state = FIND_BALL
    thresh = green
    print(state)
    print(thresh)
      
    # Keep looping
    while True:
        # Grab the current frame
        (grabbed, frame) = camera.read()
	# Resize the frame, blur it, and convert it to the HSV color space
	frame = imutils.resize(frame, width=600)
	blurred = cv2.GaussianBlur(frame, (11, 11), 0)
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	# State 1: Detect what color ball is
	if state == FIND_BALL:
            if detectBall(frame,hsv,green):
                thresh = green
                state = FOUND_BALL
            elif detectBall(frame,hsv,red):
                thresh = red
                state = FOUND_BALL
            elif detectBall(frame,hsv,yellow):
                thresh = yellow				
                state = FOUND_BALL
				
	# State 2: Ball color is found, send command to Arduino
	elif state == FOUND_BALL:
	    if thresh == green:
		# cmd: rotate CW
		print('Green = rotate CW')
		ser.write('100')
	    elif thresh == red:
		# cmd: rotate CCW
		print('Red = rotate CCW')
		ser.write('200')
	    elif thresh == yellow:
		# cmd: go up
		print('Yellow = go up')
		ser.write('300')
	    state = SENT_CMD
		
	# State 3: Sent command to Arduino, waiting for ball to disappear
	elif state == SENT_CMD:
	    if not detectBall(frame,hsv,thresh):
                state = FIND_BALL

	# Show the frame to our screen
	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF
        
	# If the 'q' key is pressed, stop the loop
	if key == ord("q"):
	    break        

    # Cleanup the camera and close any open windows
    camera.release()
    cv2.destroyAllWindows() 

if __name__ == "__main__":
	main()

          
                
