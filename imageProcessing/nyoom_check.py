def isDeepEnough(ballFound):
    near = False
    tooHigh = False
    
    near_thresh = 20
    height_thresh = 150
    
    (x1,y1,rad1) = ballFound[0]
    (x2,y2,rad2) = ballFound[1]
    
    # Nyoom is near wall if true
    if rad1 >= near_thresh or rad2 >= near_thresh:
        # Nyoom is higher/above line to cross
        if (y1+rad1) > height_thresh and (y2+rad2) > height_thresh:
            return False
        else:
            return True
         
    # Nyoom is still far from wall
    else:
        return True
            
def isAboveTable(ballFound):
    near = False
    tooLow = False

    near_thresh = 
    height_thresh = 

    (x1,y1,rad1) = ballFound[0]
    (x2,y2,rad2) = ballFound[1]
    
    # Nyoom is near table
    if rad1 >= near_thresh or rad2 >= near_thresh:    
        # Nyoom is too low to pass through table
        if (y1-rad1) < height_thresh and (y2-rad2) < height_thresh:
            return False
        else: 
            return True
    
    # Nyoom is far from table 
    # (unlikely,this will happen though, I think)
    else:
        return True
        
def isTableCentered(ballFound):
    centered = False
    
    near_thresh = 
    center_pt = 320 # middle of horizontal frame
    center_thresh = 30   
    
    (x1,y1,rad1) = ballFound[0]
    (x2,y2,rad2) = ballFound[1]
    ball_center = (x1+x2)/2
    
     # Nyoom is near table
    if rad1 >= near_thresh or rad2 >= near_thresh:    
        # Nyoom is around center of table
        if abs(ball_center-center_pt) > center_thresh:
            return False
        else: 
            return True
    
    # Nyoom is far from table 
    # (unlikely,this will happen though, I think)
    else:
        return True

        
def sortBallBoundary(ballFound):
# Sorts which two balls are on top, adn the other two are at bottom