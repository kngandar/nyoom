# Import necessary packages
from collections import deque
from time import sleep
from inputs import get_gamepad
from nyoom_xbox import isXboxUsed
from nyoom_check import isDeepEnough
import numpy as np
import argparse
import imutils
import cv2
import serial

### CONSTANTS ###
# Define HSV range of colors (lower, upper)
green = [(29, 86, 6), (64, 255, 255)]
yellow = [(10, 120, 120), (50, 255, 255)]
red = [(145, 110, 120), (195, 255, 255)]

fontColor = (0, 165, 255)        # orange
outlineColor = (0, 255, 255)     # yellow

OB_1 = False
OB_2 = False
OB_3 = False
OB_4 = False

STARTING = 10
TOO_HIGH = 11
MOVING   = 12

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

            if rad > 10:
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
    arduino = serial.Serial('/dev/tty/ACMO',115200)

    greenFound = False
    state1 = STARTING

    # Script on standby until user press "Enter"
    raw_input("Press Enter to start")

    # In autonomous mode
    print("AUTONOMOUS MODE")
    while True:
        # Switch to RC mode if xbox is used
        events = get_gamepad()
        if isXboxUsed(events) == "None":
            break

        # Grab the current frame
        (grabbed, frame) = camera.read()

        # Resize the frame, blur it, and convert it to the HSV color space
        frame = imutils.resize(frame, width=600)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # OBSTACLE 1: Go below hanging obstacle
        numBalls = 2
        if (OB_1 == False):
            color = green
            [frame,ballFound] = detectBalls(frame,hsv,color,numBalls)

            # Found 2 points
            if (len(ballFound) == numBalls) and (not greenFound):
                greenFound   = True
                (x1,y1,rad1) = ballFound[0]
                (x2,y2,rad2) = ballFound[1]
                print("Ball 1: " + str(ballFound[0]) + " Ball 2: " + str(ballFound[1]))

                # If nyoom is not deep enough, go down a bit
                if not (isDeepEnough(ballFound)):
                    print("Not deep enough")
                    arduino.write("goDown")
                    sleep(0.5)                      # seconds
                    state1 = TOO_HIGH
                else:
                    # If deep enough, send go forward command
                    if state1 == STARTING:
                        print("Deep enough")
                        arduino.write("Y_Servo_Middle")
                        sleep(0.5)                  # seconds
                        arduino.write("A_Motor_Forward")
                        sleep(0.5)                  # seconds
                        state1 = MOVING
                    # Else, do nothing, and keep going forward

            # If green balls were previously detected and then no longer
            # Assume nyoom has passed obstacle 1
            elif (len(ballFound) < numBalls) and (greenFound):
                print("Obstacle 1 complete")
                OB_1 = True
                arduino.write("DD_Motor_Stop")
                sleep(0.5)      # seconds

        # OBSTACLE 2: Go above table
#        elif (OB_2 == False):
#            color = red
#            [frame,ballFound] = detectBalls(frame,hsv,color,numBalls)

            # Found 2 closest points
#            if (len(ballFound) == numBalls) and (not redFound):
#                redFound = True
#                (x1,y1,rad1) = ballFound[0]
#                (x2,y2,rad2) = ballFound[1]
#                print("Ball 1: " + str(ballFound[0]) + " Ball 2: " + str(ballFound[1])

        # Tested detect code
        else:
            [frame,ballFound] = detectBalls(frame,hsv,green,numBalls)

            if len(ballFound) > 0:
                print("Number of balls detected: " + str(len(ballFound)))

            if len(ballFound) == 2:
                pt1_x = (ballFound[0])[0]
                pt1_y = (ballFound[0])[1]
                pt1_rad = (ballFound[0])[2]
                pt2_x = (ballFound[1])[0]
                pt2_y = (ballFound[1])[1]
                pt2_rad = (ballFound[1])[2]
                cv2.line(frame, (int(pt1_x),int(pt1_y-pt1_rad)), (int(pt2_x),int(pt2_y-pt2_rad)), outlineColor, thickness=-1)
                cv2.line(frame, (int(pt1_x),int(pt1_y+pt1_rad)), (int(pt2_x),int(pt2_y+pt2_rad)), outlineColor, thickness=-1)

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


    # Switched to RC mode
    print("REMOTE CONTROL MODE")
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
        (frame,ballFound) = detectBalls(frame,hsv,green,numBalls)

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
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

