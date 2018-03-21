def isDeepEnough(ballFound):
    near_thresh = 20
    height_thresh = 150
    
    (x1,y1,rad1) = ballFound[0]
    (x2,y2,rad2) = ballFound[1]
    
    # Nyoom is near wall if true
    if rad1 >= near_thresh and rad2 >= near_thresh:
        # Nyoom is higher/above line to cross
        if (y1-rad1) > height_thresh or (y2-rad2) > height_thresh:
            return False
        else:
            return True
         
    # Nyoom is still far from wall
    else:
        print("how did u get this deep?")
        
def isTableCentered(ballFound):    
    near_thresh = 20        # radius of ball to be considered near
    center_pt = 300         # middle of horizontal resized fram (600)
    center_thresh = 30      # range from center_pt that is still considered middle of table
    
    (x1,y1,rad1) = ballFound[0]
    (x2,y2,rad2) = ballFound[1]
    ball_cntrPt = (x1+x2)/2
    
     # Nyoom is near table
    if rad1 >= near_thresh and rad2 >= near_thresh:    
        # Nyoom is around center of table
        if abs(ball_cntrPt-center_pt) > center_thresh:
            if ball_cntrPt-center_pt) > 0:
            # Nyoom is to right of table so ...
                return 20  # GO_LEFT
            elif ball_cntrPt-center_pt < 0:
            # Nyoom is to left of table so ...
                return 21  # GO_RIGHT
        else: 
            return 0       # CENTERED 
    
    # Nyoom is far from table 
    # (unlikely,this will happen though, I think)
    else:
        print("how did u stray this far?")
        
def isAboveTable(ballFound):
    near_thresh = 20
    height_thresh = 350

    (x1,y1,rad1) = ballFound[0]
    (x2,y2,rad2) = ballFound[1]
    
    # Nyoom is near table
    if rad1 >= near_thresh and rad2 >= near_thresh:    
        # Nyoom is too low to pass through table
        if (y1-rad1) < height_thresh or (y2-rad2) < height_thresh:
            return False
        else: 
            return True
    
    # Nyoom is far from table 
    # (unlikely,this will happen though, I think)
    else:
        print("how did u get this high?")

def checkOpening(ballFound):
    near_thresh = 15
    center_pt = 300
    center_thresh = 40
            