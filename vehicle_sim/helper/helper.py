import math
import numpy as np
import matplotlib.path as mpl_path 

def pi_2_pi(angle):
    while(angle > math.pi):
        angle = angle - 2.0 * math.pi

    while(angle < -math.pi):
        angle = angle + 2.0 * math.pi

    return angle

def rotate(x, y, rad, origin=(0, 0), relative=True):
    '''
    Rotate a vector of points around a given point.
    
    ================  ==================================================
    **Arguments:**
    x                 (float) x coordinates of the points to rotate
    y                 (float) y coordinates of the points to rotate
    rad               (float) angle to rotate the [x,y] points by
    origin            ([float,float]) optional input to speicify origin of rotation
    relative          (boolean) if set to True, do not tralate the [x,y] point back to origin after rotation

    **Returns:**
    array             (numpy array) N-by-2 array of rotated points
    ================  ==================================================
    '''

    rad = -rad
    offset_x, offset_y = origin

    x = np.array(x)
    y = np.array(y)

    adjusted_x = (x - offset_x)
    adjusted_y = (y - offset_y)
    cos_rad = np.cos(rad)
    sin_rad = np.sin(rad)
    if relative == True:
        # If relative == True, do not tranlate the rotated pts back to the origin
        offset_x = 0
        offset_y = 0
    qx = offset_x + cos_rad * adjusted_x + sin_rad * adjusted_y
    qy = offset_y + -sin_rad * adjusted_x + cos_rad * adjusted_y

    return np.array([qx,qy]).T

def find_points_in_polygon(polygon, points):
    '''
    Find points that are contained within a polygon.
    
    ================  ==================================================
    **Arguments:**
    polygon           (list) list of (x,y) points which describe the polygon
    points            (list) list of (x,y) points to be evaluated  
    
    **Returns:**
    result            (list) list of (x,y) points that fall under the polygon
    ================  ==================================================
    '''
    # polygon = mpl_path.Path([[0,0],[0,1],[1,1],[1,0]])
    # points = [[0.5,0.5],[2,2],[0.2,0.9]]
    poly = mpl_path.Path(polygon)
    points = np.array(points)
    inside = poly.contains_points(points)
    # inside = np.where(poly.contains_points(points))[0]
    # return [points[i] for i in inside] 
    return np.compress(inside,points,axis=0), inside

def points_from_vec(x,y,z=None):
    '''
    Return 2D or 3D points in numpy array format from vector of lists
    
    ================  ==================================================
    **Arguments:**
    x                 (list) 1-by-N list of x coordinates
    y                 (list) 1-by-N list of y coordinates
    
    **Returns:**
    array             (numpy array) N-by-2 or N-by-3 points
    ================  ==================================================
    '''
    arr = []
    if z is None:
        arr = np.array([x,y]).T
    else:
        arr = np.array([x,y.z]).T
    return arr