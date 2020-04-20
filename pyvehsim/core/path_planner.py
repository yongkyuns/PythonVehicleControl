'''
.. module:: path_planner
   :synopsis: Perception and path planning module
.. moduleauthor:: Yongkyun Shin <github.com/yongkyuns>

This module implements simulated perception (e.g. lane detection from camera) and basic path planning algorithms.
'''

import sys
# sys.path.append('.../PythonRobotics')
from PythonRobotics.PathPlanning.CubicSpline import cubic_spline_planner

import math
import numpy as np
from pyvehsim.helper.helper import pi_2_pi, rotate, points_from_vec, find_points_in_polygon

SEARCH_RADIUS = 1000   # [m]. Search distance for finding closest path point in successive time step
MIN_DETECTION_PTS = 10  # [no unit] Number of sample points to be detected by the camera for path detection
PATH_FIT_ORDER = 3     # [no unit] Order of the polyfit used for detected path

class Camera:
    '''
    Temporary class to mimic camera detection of lanes or paths. This class receives path points, 
    then calculate lcoal path in the vehicle cooridnate frame.
    '''
    def __init__(self):
        self.view_range = 60       # [meters] furthest distance that camera 'sees'
        self.view_blind_range = -3  # [meters] camera only sees what's beyond this distance
        self.view_width_near = 6   # [meters] width of the camera field-of-view at the nearest distance (view_blind_range)
        self.view_width_far = 15   # [meters] width of the camera field-of-view at view_range
        self.field_of_view = self.get_FOV_polygon()

    def get_FOV_polygon(self):
        '''
        Compute (x,y) coordinates of corners that construct field-of-view from the top view geometry
        '''
        near_left = [self.view_blind_range, self.view_width_near/2]
        near_right = [self.view_blind_range, -self.view_width_near/2]
        far_left = [self.view_range, self.view_width_far/2]
        far_right = [self.view_range, -self.view_width_far/2]
        return np.array([near_left, far_left, far_right, near_right])

    def detect_points(self, path, ego_x, ego_y, ego_ang):
        '''
        Detect path coordinates that lie within the field-of-view of the camera.
        
        ===================  ==================================================================
        **Arguments:**
        path                 (ndarray) N-by-2 array of (x,y) cooridnates for the path
        ego_x                (float) x position of the vehicle [meters]
        ego_y                (float) y position of the vehicle [meters]
        ego_ang              (float) heading angle of the vehicle [rad]
        
        **Variables:**
        field_of_view        (list) 4 element (x,y) coordinates of FOV polygon
        
        **Returns:**
        local_detected       (ndarray) N-by-2 array of (x,y) coordinates detected
        idxs                 (list) boolean list of indexs which correspond to detected points
        ===================  ==================================================================
        '''
        global_fov = rotate(self.field_of_view[:,0]+ego_x,self.field_of_view[:,1]+ego_y,ego_ang, origin=(ego_x,ego_y), relative=False)
        detected,idxs = find_points_in_polygon(global_fov, path)
        local_detected = rotate(detected[:,0],detected[:,1],-ego_ang, origin=(ego_x,ego_y), relative=True)
        return local_detected, idxs

    def fit_polynomial(self,points):
        '''
        Fit polynomial onto the input (x,y) points
        
        ================  ==============================================================
        **Arguments:**
        points            (ndarray) N-by-2 array of (x,y) points
        
        **Returns:**
        coeff             (ndarray) Fitted polynomial coefficients, highest order first 
        ================  ==============================================================
        '''
        coeff = np.zeros(PATH_FIT_ORDER+1)
        if points.shape[0] < MIN_DETECTION_PTS:
            coeff = np.polyfit(points[:,0],points[:,1],PATH_FIT_ORDER)
        return coeff

class Planner:
    '''
    Perception and path planning class. This class takes waypooints as input and generats smooth
    path (currently cubic-spline from PythonRobotics). Also, this class acts as a basic perception module in that it calculates local 'detected'
    path from the global path, which simulates the functionality of perception stack. From the local
    path, the reference states (e.g. lateral position & heading along the path) can be computed for
    control.
    
    ================  ====================================================
    **Arguments:**
    ax                (list) Floats specifying x coordinates of waypoints
    ay                (list) Floats specifying y coordinates of waypoints
    res               (float) Spatial resolution [m] of path points 
    ================  ====================================================
    '''
    def __init__(self,ax,ay,res=0.1):
        self._spline = cubic_spline_planner.Spline2D(ax,ay)
        self.x, self.y, self.yaw, self.k,_ = cubic_spline_planner.calc_spline_course(ax,ay,res)
        self._res = res
        self.path_points = points_from_vec(self.x,self.y)
        self.camera = Camera()

    def calc_nearest_index(self, x_pos, y_pos, search_mid_pt=0, search_range=5):
        '''
        Calculate the index & distance of the closest path point from the current location specified in the input.
        
        ================  ==================================================
        **Arguments:**
        x_pos             (list) global x coordinate of current position
        y_pos             (list) global y coordinate of current position
        search_mid_pt     (int) reference index to include in search (0 is at current vehicle position)
        search_range      (float) distance [m] to perform forward/backward search from 'search_mid_pt'
        
        **Returns:**
        ind               (int) index of the closest path coordinate from the global path 
        mind              (float) distance to the closest path cooridnate
        ================  ==================================================
        '''
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

    def detect_local_path_with_FOV(self, x_offset, y_offset, ang):
        '''
        Find points from path coordiates which lie within the field-of-view of the camera.
        This function mimics detected point-cloud of lanes/paths without running the actual
        perception. If no path is detected within the FOV, returns 0 valued lists.
        
        ================  ==================================================
        **Arguments:**
        x_offset          (float) x position of the vehicle [meteres]
        y_offset          (float) y position of the vehicle [meteres]
        ang               (float) heading angle the vehicle [rad]
        
        **Variables:**
        self.path_points  (ndarray) N-by-2 or N-by-3 array of path coordinates
        
        **Returns:**
        rel_x             (list) x coordinates of the detected path
        rel_y             (list) y coordinates of the detected path
        rel_yaw           (list) angle of the detected path
        ================  ==================================================
        '''
        detected_pts, idxs = self.camera.detect_points(self.path_points,x_offset,y_offset,ang)
        rel_x = detected_pts[:,0]
        rel_y = detected_pts[:,1]
        # rel_yaw = [yaw - ang for yaw in self.yaw[start_idx:end_idx]]
        rel_yaw = np.compress(idxs,np.array(self.yaw))

        if np.sum(idxs) < MIN_DETECTION_PTS:
            # path_length = self.camera.view_range - self.camera.view_blind_range
            # sample_num = np.round(path_length / self._res)
            # rel_x = list(np.linspace(self.camera.view_blind_range,self.camera.view_range,MIN_DETECTION_PTS))
            rel_x = [0] * MIN_DETECTION_PTS
            rel_y = [0] * MIN_DETECTION_PTS
            rel_yaw = [0] * MIN_DETECTION_PTS
        return list(rel_x), list(rel_y), list(rel_yaw)
    
    def detect_local_path(self, x_offset, y_offset, ang):
        '''
        Mimic camera detection of path by generating relative path to follow with respect to vehicle coordinate.
        This function takes global path coordinates in x, y, and heading and generates relative path with respect
        to vehicle. 
        
        ================  ==================================================
        **Arguments:**
        x_offset          (float) vehicle global x coordinate [m] 
        y_offset          (float) vehicle global y coordinate [m] 
        ang               (float) vehicle heading angle [rad] 
        
        **Variables:**
        x                 (list) global x coordinates of desired path
        y                 (list) global y coordinates of desired path
        yaw               (list) heading angle [rad] of path at [self.x, self.y] points
        
        **Returns:**
        rel_x             (list) x coordinates of path w.r.t. vehicle
        rel_y             (list) y coordinates of path w.r.t. vehicle
        rel_yaw           (list) desired heading angle w.r.t. vehicle
        ================  ==================================================
        '''

        search_range = int(SEARCH_RADIUS / self._res)
        idx,_ = self.calc_nearest_index(x_offset,y_offset,search_range=search_range)
        start_idx = max(0,idx - 10)
        end_idx = min(len(self.x), idx + 600)

        xy_pts = rotate(self.x[start_idx:end_idx],self.y[start_idx:end_idx], -ang, origin=(x_offset,y_offset))
        rel_x = [pt[0] for pt in xy_pts]
        rel_y = [pt[1] for pt in xy_pts]
        rel_yaw = [yaw - ang for yaw in self.yaw[start_idx:end_idx]]
        return rel_x, rel_y, rel_yaw
    
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

    # plt.figure()
    # # plt.plot(ax,ay)
    # plt.plot(planner.x,planner.y)
    # plt.plot(x_pts,y_pts)
    # plt.plot(x,y, "ok", label="car")
    # # plt.plot(planner.x[idx],planner.y[idx], "xk", label="nearest")
    # plt.axis("equal")
    # plt.grid(True)
    # plt.legend()
    # plt.show()

    

if __name__ == '__main__':
    main()


