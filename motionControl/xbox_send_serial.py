#!/usr/bin/env python
# coding: Latin-1
# Load library functions we want

import serial
from inputs import get_gamepad

ser = serial.Serial('/dev/ttyACM0',9600) 

try:
    print('Press CTRL+C to quit')

    # Loop indefinitely

    while True:

        events = get_gamepad()

        for event in events:

            #print(event.code, event.state)
            if event.code == "BTN_TL":

                if event.state == True:

                    print("LB_Servo_Up")
                    ser.write("LB_Servo_Up")

            if event.code == "BTN_TR":

                if event.state == True:

                    print("RB_Servo_Down")
                    ser.write("RB_Servo_Down")

            if event.code == "BTN_NORTH":

                if event.state == True:
                    print("X_Panic_Button")
                    ser.write("X_Panic_Button")
                    

            if event.code == "BTN_SOUTH":

                if event.state == True:

                    print("A_Motor_Forward")
                    ser.write("A_Motor_Forward")

            if event.code == "BTN_EAST":

                if event.state == True:

                    print("B_Motor_Backward")
                    ser.write("B_Motor_Backward")

            if event.code == "ABS_HAT0Y":

                if event.state == -1:

                    print("DU_Front")
                    ser.write("DU_Front")

                elif event.state == 1:

                    print("DD_Motor_Stop")
                    ser.write("DD_Motor_Stop")

            if event.code == "ABS_HAT0X":

                if event.state == -1:

                    print("DL_Left")
                    ser.write("DL_Left")

                elif event.state == 1:

                    print("DR_Right")
                    ser.write("DR_Right")

            if event.code == "BTN_WEST":

                if event.state == True:

                    print("Y_Servo_Middle")
                    ser.write("Y_Servo_Middle")
                    

            #print("#### End ####")

           # print(event.ev_type, event.code, event.state)

except KeyboardInterrupt:

    # CTRL+C exit, disable all drives

    print("stop")

print("bye")
