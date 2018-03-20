def isXboxUsed(events):
    for event in events:
        command = "None"    # not sure if this should go here or not
        
        if event.code == "BTN_TL":
            if event.state == True:
                command = "LB" #Servo_Up 

        if event.code == "BTN_TR":
            if event.state == True:
                command = "RB" #Servo_Down

        if event.code == "BTN_NORTH":
            if event.state == True:
                command = "X" #Panic_Button

        if event.code == "BTN_SOUTH":
            if event.state == True:
                command = "A" #Motor_Forward

        if event.code == "BTN_EAST":
            if event.state == True:
                command = "B" #Motor_Backward

        if event.code == "ABS_HAT0Y":
            if event.state == -1:
                command = "DU" #Front

            elif event.state == 1:
                command = "DD" #Motor_Stop

        if event.code == "ABS_HAT0X":
            if event.state == -1:
                command = "DL" #Left 

            elif event.state == 1:
                command = "DR" #Right

        if event.code == "BTN_WEST":
            if event.state == True:
                command = "Y" #Servo_Middle
                
        if command is not "None":
            print(command)
            
    return command
                    
