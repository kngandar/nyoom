def isXboxUsed(events):
    for event in events:
        command = "None"    # not sure if this should go here or not
        
        if event.code == "BTN_TL":
            if event.state == True:
                command = "LB_Servo_Up"

        if event.code == "BTN_TR":
            if event.state == True:
                command = "RB_Servo_Down"

        if event.code == "BTN_NORTH":
            if event.state == True:
                command = "X_Panic_Button"

        if event.code == "BTN_SOUTH":
            if event.state == True:
                command = "A_Motor_Forward"

        if event.code == "BTN_EAST":
            if event.state == True:
                command = "B_Motor_Backward"

        if event.code == "ABS_HAT0Y":
            if event.state == -1:
                command = "DU_Front"

            elif event.state == 1:
                command = "DD_Motor_Stop"

        if event.code == "ABS_HAT0X":
            if event.state == -1:
                command = "DL_Left"

            elif event.state == 1:
                command = "DR_Right"

        if event.code == "BTN_WEST":
            if event.state == True:
                command = "Y_Servo_Middle"
                
    print(command)
    return command
                    
