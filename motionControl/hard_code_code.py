#!/usr/bin/env python
# coding: Latin-1
# Load library functions we want

import serial
from time import sleep
from inputs import get_gamepad

ser = serial.Serial('/dev/ttyACM0',115200)

raw_input("Press Enter to continue")
ser.write("1")
print('Arduino is doing thing')
sleep(25)

try:
    print('Entering RC mode...')
    print('Press CTRL+C to quit')

    # Loop indefinitely

    while True:

        events = get_gamepad()

        for event in events:

            #print(event.code, event.state)
            if event.code == "BTN_TL":

                if event.state == True:

                    print("LB_Servo_Up")
                    ser.write("LB")

            if event.code == "BTN_TR":

                if event.state == True:

                    print("RB_Servo_Down")
                    ser.write("RB")

            if event.code == "BTN_NORTH":

                if event.state == True:
                    print("X_Panic_Button")
                    ser.write("X")
                    

            if event.code == "BTN_SOUTH":

                if event.state == True:

                    print("A_Motor_Forward")
                    ser.write("A")

            if event.code == "BTN_EAST":

                if event.state == True:

                    print("B_Motor_Backward")
                    ser.write("B")

            if event.code == "ABS_HAT0Y":

                if event.state == -1:

                    print("DU_SouthEast")
                    ser.write("DU")

                elif event.state == 1:

                    print("DD_Middle")
                    ser.write("DD")

            if event.code == "ABS_HAT0X":

                if event.state == -1:

                    print("DL_Left")
                    ser.write("DL")

                elif event.state == 1:

                    print("DR_Right")
                    ser.write("DR")

            if event.code == "BTN_WEST":

                if event.state == True:

                    print("Y_Servo_Middle")
                    ser.write("Y")
                    

            #print("#### End ####")

           # print(event.ev_type, event.code, event.state)

except KeyboardInterrupt:

    # CTRL+C exit, disable all drives

    print("stop")

print("bye")
