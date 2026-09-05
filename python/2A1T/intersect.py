import math

def intersect(distances, points=[[0,0], [0,10]]): 
    r1, r2 = distances 
    p1, p2 = points 
    x1, y1 = p1 
    x2, y2 = p2 

    mul = r1**2 - r2**2 - y1**2 + y2**2 
    div = -2*y1 + 2*y2 
    
    # Safety check: Prevent divide by zero if anchors have the same Y coordinate
    if div == 0:
        raise ValueError("Anchors cannot be at the same location")
        
    y = mul / div 
    
    # Calculate the value inside the square root
    inner_val = r1**2 - (y - y1)**2
    
    # Handle non-intersecting circles
    if inner_val < 0:
        # The circles are too small to touch. 
        # The closest physical estimate is directly on the y-axis.
        x = 0
    else:
        x = math.sqrt(inner_val)

    return (x, y) 

