

import sys
sys.path.append('.')
from PythonRobotics.PathPlanning.CubicSpline import cubic_spline_planner

import math
from helper import *

SEARCH_RADIUS = 1000 #[m]. Search distance for finding closest path point in successive time step

class Planner:
    '''
    Planner receives waypoints and generates desired path
    '''
    def __init__(self,ax,ay,res=0.1):
        self._spline = cubic_spline_planner.Spline2D(ax,ay)
        self.x, self.y, self.yaw, self.k,_ = cubic_spline_planner.calc_spline_course(ax,ay,res)
        self._res = res
        

    def calc_nearest_index(self, x_pos, y_pos, search_mid_pt=0, search_range=5):
        """
        Calculate the index & distance of the closest path point from the current location specified in the input.

        Input:
            x_pos         --> global x coordinate of current position
            y_pos         --> global y coordinate of current position
            search_mid_pt --> reference index to include in search (0 is at vehicle position)
            search_range  --> range of index to perform search from 'search_mid_pt'
        
        Output:
            ind           --> index of the closest path coordinate from the global path 
            mind          --> distance to the closest path coordinate
        """
        search_begin_idx = max(0,search_mid_pt - search_range)
        search_end_idx = min(len(self.x) - 1, search_mid_pt + search_range)

        dx = [x_pos - icx for icx in self.x[search_begin_idx:search_end_idx]]
        dy = [y_pos - icy for icy in self.y[search_begin_idx:search_end_idx]]

        d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

        mind = min(d)

        ind = d.index(mind) + search_begin_idx

        mind = math.sqrt(mind)

        dxl = self.x[ind] - x_pos
        dyl = self.y[ind] - y_pos

        angle = pi_2_pi(self.yaw[ind] - math.atan2(dyl, dxl))
        if angle < 0:
            mind *= -1

        return ind, mind    
    
    def detect_local_path(self, x_offset, y_offset, ang):
        """
        Mimic camera detection of path by generating relative path to follow with respect to vehicle coordinate.
        This function takes global path coordinates in x, y, and heading and generates relative path with respect
        to vehicle. 

        Input:
            self.x   --> global x coordinates of desired path
            self.y   --> global y coordinates of desired path
            self.yaw --> heading angle of path at [self.x, self.y] points
            x_offset --> vehicle x coordinate [m] (global)
            y_offset --> vehicle y coordinate [m] (global)
            ang      --> vehicle heading angle [rad] (global) 

        Output:
            rel_x    --> x coordinate of path near vehicle in vehicle-centered coordinate frame
            rel_y    --> y coordinate of path near vehicle in vehicle-centered coordinate frame
            rel_yaw  --> relative angles of path coordinates (i.e. vehicle heading angle is 0)
        """

        search_range = int(SEARCH_RADIUS / self._res)
        idx,_ = self.calc_nearest_index(x_offset,y_offset,search_range=search_range)
        start_idx = max(0,idx - 10)
        end_idx = min(len(self.x), idx + 600)

        xy_pts = [self.rotate(x,y,-ang,origin=(x_offset,y_offset)) for (x,y) in zip(self.x[start_idx:end_idx],self.y[start_idx:end_idx])]
        rel_x = [pt[0] for pt in xy_pts]
        rel_y = [pt[1] for pt in xy_pts]
        rel_yaw = [yaw - ang for yaw in self.yaw[start_idx:end_idx]]
        return rel_x, rel_y, rel_yaw
        
    def rotate(self, x, y, rad, origin=(0, 0), relative=True):
        """
        Rotate a point around a given point.

        Input:
            x        --> x coordinate of the point to rotate
            y        --> y coordinate of the point to rotate
            rad      --> angle to rotate the [x,y] point by
            origin   --> optional input to speicify origin of rotation 
            relative --> if set to True, do not tralate the [x,y] point back to origin after rotation

        Output:
            qx       --> rotated x coordinate of input [x,y]
            qy       --> rotated y coordinate of input [x,y]
        """
        rad = -rad
        offset_x, offset_y = origin

        adjusted_x = (x - offset_x)
        adjusted_y = (y - offset_y)
        cos_rad = math.cos(rad)
        sin_rad = math.sin(rad)
        if relative == True:
            # If relative == True, do not tranlate the rotated pts back to the origin
            offset_x = 0
            offset_y = 0
        qx = offset_x + cos_rad * adjusted_x + sin_rad * adjusted_y
        qy = offset_y + -sin_rad * adjusted_x + cos_rad * adjusted_y
        return qx, qy
    
    def calc_ref(self, x, y, yaw, step, N):
        x_ref = []
        pos_ref = []
        yaw_ref = []
        
        # stepIdxSize = int(step / self._res)

        # temp_pos_ref = []
        # temp_yaw_ref = []
        for i in range(N):
            x_val = min(x, key=lambda x:abs(x-i*step))
            idx = x.index(x_val)
            x_ref.append(x[idx])
            pos_ref.append(y[idx])
            yaw_ref.append(yaw[idx])

            # temp_pos_ref.append(y[stepIdxSize*i])
            # temp_yaw_ref.append(yaw[stepIdxSize*i])
        return x_ref, pos_ref, yaw_ref

def main():
    import matplotlib.pyplot as plt
    import math

    # ax = [0.0, 60.0, 125.0, 50.0, 75.0, 30.0, -10.0]
    # ay = [0.0, 0.0, 50.0, 65.0, 30.0, 50.0, -20.0]

    ax = [0, 100, 200, 300, 400, 500]
    ay = [0, 0, 30, 60, 60, 60]

    planner = Planner(ax,ay)

    x = 125
    y = 40
    yaw = 20/180*math.pi
    
    x_pts, y_pts, yaws = planner.detect_local_path(x,y,yaw)

    x_ref, pos_ref, yaw_ref = planner.calc_ref(x_pts, y_pts, yaws, 3, 5)

    plt.figure()
    # plt.plot(ax,ay)
    plt.plot(planner.x,planner.y)
    plt.plot(x_pts,y_pts)
    plt.plot(x,y, "ok", label="car")
    # plt.plot(planner.x[idx],planner.y[idx], "xk", label="nearest")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.show()

if __name__ == '__main__':
    main()


