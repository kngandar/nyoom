# Import necessary packages
from collections import deque
import numpy as np
import argparse
import imutils
import cv2

# Construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
                help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
                help="max buffer size")
args = vars(ap.parse_args())

# Define HSV range of colors
#greenLower = (10, 120, 120)
#greenUpper = (50, 255, 255)
#greenLower = (145, 110, 120)
#greenUpper = (195, 255, 255)
lower = (29, 86, 6)
upper = (64, 255, 255)
lower = (145, 110, 120)
upper = (195, 255, 255)
#redLower =
#redUpper =
#yellowLower =
#yellowUpper = 
#pts = deque(maxlen=args["buffer"])

# If a video path was not supplied, grab the reference to the webcam
if not args.get("video", False):
    camera = cv2.VideoCapture(0)

# Otherwise, grab a reference to the video file
else:
    camera = cv2.VideoCapture(args["video"])            

# Keep looping
while True:
    # Grab the current frame
    (grabbed, frame) = camera.read()

    # If we are viewing a video and we did not grab a frame,
    # then we hae reached the end of the video
    if args.get("video" ) and not grabbed:
        break

    # Resize the frame, blur it, and convert it to the HSV color space
    frame = imutils.resize(frame, width=600)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # Construct a mask for the color "green", then perform a series of
    # dilations and erosions to remove any small blobs left in the mask
    mask = cv2.inRange(hsv, lower, upper)
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
        center2 = None
        cnts2 = [e for e in cnts if e not in (c)]

        # Ball 2 if found
        if len(cnts2) > 0:
            c2 = max(cnts2, key=cv2.contourArea)
            ((a,b), rad2) = cv2.minEnclosingCircle(c2)
            M = cv2.moments(c2)
            center2 = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            if rad2 > 10:
                #print("Ball 2")
                cv2.circle(frame, (int(a), int(b)), int(rad2), (0, 255, 255), 2)
                cv2.putText(frame, "r: " + str(int(rad2)), (int(a-rad2),int(b-rad2)),cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv2.putText(frame, "x,y: " + str(int(a)) + ", " +  str(int(b)) , (int(a-rad2),int(b-rad2-rad2)),cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        ((x,y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # Only proceed if the radius meets a minimum size
        if radius > 10:
            #print("Ball 1")
            # Draw the circle and centroid on the frame
            # then update the list of tracked points
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)
            cv2.putText(frame, "r: " + str(int(radius)), (int(x-radius),int(y-radius)),cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.putText(frame, "x,y: " + str(int(x)) + ", " +  str(int(y)) , (int(x-radius),int(y-radius-radius)),cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
           
 

    # Show the frame to our screen
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF

    # If the 'q' key is pressed, stop the loop
    if key == ord("q"):
        break
    elif key == ord("g"):
        print 'Set green thresh'
        lower = (29, 86, 6)
        upper = (64, 255, 255)
    elif key == ord("r"):
        print 'Set red thresh'
        lower = (145, 110, 120)
        upper = (195, 255, 255)
    elif key == ord("y"):
        print 'Set yellow thresh'
        lower = (10, 120, 120)
        upper = (50, 255, 255)
        
    

# Cleanup the camera and close any open windows
camera.release()
cv2.destroyAllWindows()

