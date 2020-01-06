'''
.. module:: visualizer
   :synopsis: Define classes required for animated visualization.
.. moduleauthor:: Yongkyun Shin <github.com/yongkyuns>

This module defines classes required for visualization.

Example:

.. code-block:: python

    import visualizer

    def update_func(): # define update-function to be called by visualizer for each time-step
        pass

    view = Visualizer(update_func)
    view.entry_point()
'''

from pyqtgraph.Qt import QtCore, QtGui
from gl_items import Box, Line, Grid, Scatter
import pyqtgraph.opengl as gl



class Visualizer():
    '''
    This class uses pyqtgraph and is a primary interface for interacting with the PyQt window.
    
    ================  ==================================================
    **Arguments:**
    update_func       (function) Function to be executed by PyQt at each time step
    name              (string) Name of the window
    refresh_rate      (float) Update rate of the view [ms]
    ================  ==================================================
    '''
    def __init__(self,update_func, name='Simulator', refresh_rate=50):
        self._app, self._window = self.init_window(name)
        self.add_grid()

        self.update_func = update_func

        self.car = Box('ego vehicle',color=(0,255,0,150),size=(4,2,1))

        self.global_path = Line('global path',color='r')

        self.local_path = Line('detected path',color='b',width=6)
        self.local_path.setParentItem(self.car)

        self.control_points = Scatter('control points',color=[1,1,0,0.8])
        self.control_points.setParentItem(self.car)

        self._window.addItem(self.car)
        self._window.addItem(self.global_path)
        self._window.addItem(self.local_path)
        self._window.addItem(self.control_points)

        self._timer = QtCore.QTimer()
        self._timer.timeout.connect(self.update)
        self._timer.start(refresh_rate)

    def entry_point(self):
        '''
        This entry point is used to trigger the PyQt application
        '''
        QtGui.QApplication.instance().exec_()

    def init_window(self,name):
        '''
        Initialize PyQt window and default camera position.
        '''
        ## Create a GL View widget to display data
        app = QtGui.QApplication([])
        w = gl.GLViewWidget() 
        w.resize(1900/2,1080/2)
        w.show()
        w.setWindowTitle(name)
        w.setCameraPosition(distance=50,elevation=90,azimuth=270) #elevation of 90deg is top view, azimuth=top-view-rotation
        return app, w
    
    def add_grid(self):
        '''
        Add grid to the view
        '''
        g = Grid()
        g.scale(5,5,5)
        self._window.addItem(g)
    
    def update(self):
        '''
        Invoke exteral update function and camera position around the object in focus
        '''
        self.update_func()
        self.update_origin()
    
    def update_origin(self):
        '''
        Update the camaera focus point
        '''
        x,y = self.car.mapToView((0,0))
        cam_center = self._window.opts['center']
        self._window.opts['center'] = QtGui.QVector3D(x, y, cam_center[2])



def update_func():
    pass

def main():
    view = Visualizer(update_func)
    view.entry_point()


if __name__ == '__main__':
    main()