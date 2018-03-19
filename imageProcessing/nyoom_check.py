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
            
 
    