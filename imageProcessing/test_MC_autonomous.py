# Make test script to try automate shit,
# Also run like camera things

# Import necessary packages
from collections import deque
from time import sleep
from inputs import get_gamepad
from nyoom_xbox import isXboxUsed
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
    color = yellow
    
    RC = False
    # Script on standby until user press "Enter"
    raw_input("Press Enter to start")
    arduino.write("DU")
        
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
        
        [frame,ballFound] = detectBalls(frame,hsv,color,numBalls)
        
        # Show the frame to our screen
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF
        
        # If the 'q' key is pressed, stop the loop
        if key == ord("q"):
            print("Quit Test")
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
            
        elif key == ord("w"):
            print 'Go forward'
            arduino.write("goFWD")
        elif key == ord("s"):
            print 'Go backward'
            arduino.write("goBWD")
        elif key == ord("a"):
            print 'Move left slightly'
            arduino.write("moveL")
            while True:
                response = arduino.readline()
                if response == "imDone":
                    print 'Nyoom moved left'
                    break                    
        elif key == ord("d"):
            print 'Move right slightly'
            arduino.write("moveR")
            while True:
                response = arduino.readline()
                if response == "imDone":
                    print 'Nyoom moved right'
                    break
            
        elif key == ord("i"):
            print 'Go up slightly'
            arduino.write("fwStop")
        elif key == ord("k"):
            print 'Go down slightly'
            arduino.write("bwStop")
        elif key == ord("j"):
            print 'Go up slightly'
            arduino.write("goUp")
        elif key == ord("l"):
            print 'Go down slightly'
            arduino.write("goDown")
            
        elif key == ord("g"):
            print 'Reorient'
            arduino.write("DU")
            while True:
                response = arduino.readline()
                if response == "IMUdone":
                    print 'Nyoom is reoriented'
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
        
        