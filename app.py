'''
.. module:: app
   :synopsis: Define classes required for the application.
.. moduleauthor:: Yongkyun Shin <github.com/yongkyuns>

This module defines classes required for the GUI application.

Example:

.. code-block:: python

    import app

    def update_func(): # define update-function to be called by app for each time-step
        pass

    app = Application(update_func)
    app.__entry_point()
'''

from simulator import Simulator
from pyqtgraph.Qt import QtCore, QtGui
from gl_items import Box, Line, Grid, Scatter
from plot_items import Signal
import pyqtgraph.opengl as gl
import pyqtgraph as pg
from pyqtgraph.dockarea import *
from graph_loader import load_graph
from gl_view import GLView
from plot_view import PlotView

WINDOW_WIDTH = 1920/2
WINDOW_HEIGHT = 1080/2

# view_graph object keys for visualization
CAR = 'car'
LOCAL_PATH = 'local_path'
CTRL_PTS = 'ctrl_pts'
GLOBAL_PATH = 'global_path'
STR_ANG = 'str_ang'
REF1_ERR = 'ref1_err'

class Application():
    '''
    This class uses pyqtgraph and is a primary interface for interacting with the PyQt window.
    
    ================  ==================================================
    **Arguments:**
    update_func       (function) Function to be executed by PyQt at each time step
    name              (string) Name of the window
    refresh_rate      (float) Update rate of the view [ms]
    ================  ==================================================
    '''
    def __init__(self, update_func=None, refresh_rate=50, dx=0.01):
        # pg.setConfigOption('leftButtonPan', False)

        self.simulator = Simulator(sim_time=1,verbose=True)
        self.log = self.simulator.run()

        self._QApp = QtGui.QApplication([])

        scene_file = 'view_graph'
        self.graph = load_graph(scene_file.split('.yaml')[0]+'.yaml')

        self._3DView = GLView(self.graph)
        self._pltView = PlotView(self.graph,dx=dx)
        self._window = self.__init_window(self._3DView, self._pltView)

        self._3DView.setFocusOnObj(self.graph[CAR])

        self.currentStep = 0

        self._timer = QtCore.QTimer()
        self._timer.timeout.connect(self.update)
        self._timer.start(refresh_rate)

        self.graph[GLOBAL_PATH].setData(x=self.simulator.vehicle.path.x,y=self.simulator.vehicle.path.y)

    def __entry_point(self):
        '''
        This entry point is used to trigger the PyQt application
        '''
        QtGui.QApplication.instance().exec_()

    def __init_window(self,glView,pltView):
        '''
        Initialize PyQt window and default camera position.
        '''
        ## Create a GL View widget to display data
        win = QtGui.QMainWindow()
        area = DockArea()
        win.setCentralWidget(area)
        win.resize(WINDOW_WIDTH,WINDOW_HEIGHT)
        win.setWindowTitle('Simulator')
 
        d1 = Dock("3D View", size=(WINDOW_WIDTH/2, WINDOW_HEIGHT))     ## give this dock the minimum possible size
        d2 = Dock("Signal", size=(WINDOW_WIDTH/2,WINDOW_HEIGHT), closable=False)
        area.addDock(d1, 'left')      ## place d1 at left edge of dock area (it will fill the whole space since there are no other docks yet)
        area.addDock(d2, 'right')     ## place d2 at right edge of dock area

        d1.hideTitleBar()
        d2.hideTitleBar()

        d1.addWidget(glView)
        d2.addWidget(pltView)

        win.show()

        return win

    def run(self):
        self.__entry_point()

    def __resetPlayback(self):
            self.currentStep = 0
            self._pltView.translateBy(-self.simulator.sim_time)

    def __updateView(self):
        i = self.currentStep

        #3D Scene object update
        log = self.log
        self.graph[CAR].setData(x=log.x[i],y=log.y[i],z_ang=log.yaw[i]*180/3.14159)
        self.graph[CTRL_PTS].setData(x=log.ctrl_pt_x[i], y=log.ctrl_pt_y[i])
        self.graph[LOCAL_PATH].setData(x=log.path_x[i], y=log.path_y[i])
        #2D Plot update
        self.graph[STR_ANG].setData(x=log.t[:i],y=log.str_ang[:i])
        self.graph[REF1_ERR].setData(x=log.t[:i],y=log.ref[:i])

    def __timeReachedEnd(self):
        return (self.currentStep * self.simulator.sample_time) >= self.simulator.sim_time

    def update(self):
        '''
        Invoke exteral update function and camera position around the object in focus
        '''
        if self.__timeReachedEnd():
            self.__resetPlayback()

        self.__updateView()
        self.currentStep += 1
        self._pltView.roll_plot()
    

def main():
    app = Application()
    app.run()


if __name__ == '__main__':
    main()