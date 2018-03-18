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

fontColor = (0, 165, 255)        # orange
outlineColor = (0, 255, 255)     # yellow

          
### FUNCTIONS ###
def detectBalls(frame,hsv,color,numBalls):
    lower = color[0]
    upper = color[1]
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    ballFound = []
    ballPosition = []
    
    for i in range(1,numBalls):
        if len(cnts) > 0:
            cMax = max(cnts, key=cv2.contourArea)
            ((x,y), rad) = cv2.minEnclosingCircle(cMax)
            print("x,y,rad:" + str([x,y,rad]))

            if rad > 10:
                cv2.circle(frame, (int(x), int(y)), int(rad), outlineColor, 2)
                cv2.putText(frame, "r: " + str(int(rad)), (int(x-rad),int(y-rad)),cv2.FONT_HERSHEY_SIMPLEX, 0.6, fontColor,2)
                cv2.putText(frame, "x,y: " + str(int(x)) + ", " +  str(int(y)) , (int(x-rad),int(y-rad-rad)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, fontColor,2)
                print("a ball")
                ballPosition = [x,y,rad]
                print("a ball's position")
                ballFound.append(ballPosition)
                print("ball found: " + str(ballPosition))
                
            else:
                print("All balls detected")
                break

            cnts = [e for e in cnts if e not in (cMax)]

    return (frame,ballFound)

def main():
    camera = cv2.VideoCapture(0)            

    # Keep looping
    while True:
        # Grab the current frame
        (grabbed, frame) = camera.read()
        
        # Resize the frame, blur it, and convert it to the HSV color space
        frame = imutils.resize(frame, width=600)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        numBalls = 3

        [frame,ballFound] = detectBalls(frame,hsv,green,numBalls)
        
        if len(ballFound) > 0:
            print("Number of balls detected: " + str(len(ballFound))) 

        # Show the frame to our screen
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF

        # If the 'q' key is pressed, stop the loop
        if key == ord("q"):
            break
        elif key == ord("g"):
            print 'Set green thresh'
            color = green
        elif key == ord("r"):
            print 'Set red thresh'
            color= red
        elif key == ord("y"):
            print 'Set yellow thresh'
            color = yellow
            
    # Cleanup the camera and close any open windows
    camera.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

