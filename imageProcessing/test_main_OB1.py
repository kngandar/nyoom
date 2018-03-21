# Import necessary packages
from collections import deque
from time import sleep
from inputs import get_gamepad
from nyoom_xbox import isXboxUsed
import nyoom_check
import numpy as np
import argparse
import imutils
import cv2
import serial

### CONSTANTS ###
# Define HSV range of colors (lower, upper)
yellow = [(10, 120, 120), (50, 255, 255)]
red = [(145, 110, 120), (195, 255, 255)]
green = [(29, 86, 6), (64, 255, 255)]

fontColor = (0, 165, 255)        # orange
outlineColor = (0, 255, 255)     # yellow

OB_1 = False
OB_2 = False
OB_3 = False
OB_4 = False

STOPPED = 10
MOVING = 11

GO_LEFT = 20
GO_RIGHT = 21


### VARIABLES ###
MIN_RAD = 15
PASS = 3 # second


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
            #print("x,y,rad:" + str([x,y,rad]))

            if rad > MIN_RAD:
                cv2.circle(frame, (int(x), int(y)), int(rad), outlineColor, 2)
                cv2.putText(frame, "r: " + str(int(rad)), (int(x-rad),int(y-rad)),cv2.FONT_HERSHEY_SIMPLEX, 0.6, fontColor,2)
                cv2.putText(frame, "x,y: " + str(int(x)) + ", " +  str(int(y)) , (int(x-rad),int(y-rad-rad)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, fontColor,2)
                ballPosition = [x,y,rad]
                ballFound.append(ballPosition)

            # All balls detected
            else:
                break

            cnts = [e for e in cnts if e not in (cMax)]

    return (frame,ballFound)

def main():
    # Initialization
    camera = cv2.VideoCapture(0)
    out = cv2.VideoWriter('output.avi', -1, 20.0, (640,480))
    arduino = serial.Serial('/dev/ttyACM0',115200)

    firstYellowFound = False
    firstRedFound = False
    firstGreenFound = False
    state1 = STOPPED
    state2 = STOPPED
    state3 = STOPPED
    center2 = False    
    center3 = False
    
    RC = False
    numBalls = 2

    # Script on standby until user press "Enter"
    raw_input("Press Enter to start")
    arduino.write("do thing")
    arduino.write("DU")

    # In autonomous mode
    print("AUTONOMOUS MODE")
    while True:
        # Switch to RC mode if xbox is used
        try:
            events = get_gamepad()
            if isXboxUsed(events) == "DU":
                print("found xbox, switch over fool")
                RC = True
                break
        except:
            pass

        # Grab the current frame
        (grabbed, frame) = camera.read()

        # Resize the frame, blur it, and convert it to the HSV color space
        frame = imutils.resize(frame, width=600)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # -------------------------------------
        # OBSTACLE 1: Go below hanging obstacle
        # -------------------------------------
        if (OB_1 == False):
            color = yellow
            [frame,ballFound] = detectBalls(frame,hsv,color,numBalls)
            
            # Check if 2 balls detected
            if len(ballFound) == numBalls:
                if not firstYellowFound:
                    firstYellowFound = True
                    
                # Whether this is first time or not, always check for height range
                if not nyoom_check.isDeepEnough(ballFound):
                    print("Not deep enough")                        
                    arduino.write("goDown")
                    state1 = STOPPED
                else:
                    # If nyoom is not moving and is deep enough, go forward
                    if state1 == STOPPED:
                        arduino.write("goFWD")
                        state1 = MOVING
                    # Else do nothing since nyoom is currently moving
                
            else:
                # No ball detected but yellow balls were already detected
                # Finished obstacle 1
                if firstYellowFound:
                    print("Obstacle 1 complete")
                    OB_1 = True
                    sleep(PASS)
                    arduino.write("fwStop")
                # No ball detected and no yellow balls have been detected at all
                # Nyoom is not close enough to obstacle to start responding (init)
                else:
                    print("Nyoom still can't see hanging wall")
                    arduino.write("goFWD")
                    sleep(1.5)    # seconds
                    arduino.write("fwStop")
                    
        # ----------------------------
        # Completed all 3 obstacles
        # Switch to RC immediately
        # ----------------------------
        else:
            RC = True
            break       
                
        # Write the frame
        out.write(frame)
        # Show the frame to our screen
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF
        
        # If the 'q' key is pressed, stop the loop
        if key == ord("q"):
            print("Quit Autonomous")
            break

            
    # Switched to RC mode
    if RC:
        print("REMOTE CONTROL MODE")
        color = yellow
        while True:
            events = get_gamepad()
            command = isXboxUsed(events)
            if command is not "None":
                arduino.write(command)

            (grabbed, frame) = camera.read()
            # Resize the frame, blur it, and convert it to the HSV color space
            frame = imutils.resize(frame, width=600)
            blurred = cv2.GaussianBlur(frame, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            numBalls = 2
            (frame,ballFound) = detectBalls(frame,hsv,color,numBalls)

            # Write the frame
            out.write(frame)
            # Show the frame to our screen
            cv2.imshow("Frame", frame)
            key = cv2.waitKey(1) & 0xFF

            # If the 'q' key is pressed, stop the loop
            if key == ord("q"):
                print("Quit RC")
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
    out.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

