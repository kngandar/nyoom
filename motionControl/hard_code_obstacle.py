from inputs import get_gamepad
from nyoom_xbox import isXboxUsed
from time import sleep
import serial

def main():
    arduino = serial.Serial('/dev/ttyACM0',115200)
    
    raw_input("Press Enter to start")
    arduino.write("do thing")
    arduino.write("DU")
    
    # Obstacle 1
    for i in range(1,3):
        arduino.write("goDown")
    arduino.write("goFWD")
    sleep(10)    # seconds
    arduino.write("fwStop")
    
    # Obstacle 2
    for i in range(1,3):
        arduino.write("goUp")
    arduino.write("goFWD")
    sleep(10)    # seconds
    arduino.write("fwStop")
    
    # Obstacle 3
    arduino.write("goFWD")
    sleep(10)
    arduino.write("fwStop")
    
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

                        print("DU_Front")
                        ser.write("DU")

                    elif event.state == 1:

                        print("DD_Motor_Stop")
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

        except KeyboardInterrupt:

        # CTRL+C exit, disable all drives

        print("stop")
        print("bye")
    