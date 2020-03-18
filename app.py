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
    app.run()
'''

from simulator import Simulator
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets
from gl_items import Box, Line, Grid, Scatter
from plot_items import Signal
import pyqtgraph.opengl as gl
import pyqtgraph as pg
# from pyqtgraph.dockarea import *
from graph_loader import load_graph
from gl_view import GLView
from plot_view import PlotView
from gui_view import GUIView
from qt_worker import Worker
import qt_dark_style

WINDOW_WIDTH = 1920/2
WINDOW_HEIGHT = 1080/2
MAX_THREAD = 2

# Miscellenious 
WINDOW_TITLE = 'Simulator'

# view_graph object keys for visualization
CAR = 'car'
LOCAL_PATH = 'local_path'
CTRL_PTS = 'ctrl_pts'
GLOBAL_PATH = 'global_path'
STR_ANG = 'str_ang'
REF1_ERR = 'ref1_err'



class Application(QtCore.QObject):
    '''
    This class uses pyqtgraph and is a primary interface for interacting with the PyQt window.
    
    ================  ==================================================
    **Arguments:**
    update_func       (function) Function to be executed by PyQt at each time step
    name              (string) Name of the window
    refresh_rate      (float) Update rate of the view [ms]
    ================  ==================================================
    '''

    sim_finished_signal = QtCore.pyqtSignal()
    sim_progress_signal = QtCore.pyqtSignal(int)

    def __init__(self, update_func=None, refresh_rate=10, dx=0.01):
        super().__init__()
        # pg.setConfigOption('leftButtonPan', False)

        self._QApp = QtGui.QApplication([]) # QApplication must be created before creating any GUI elements
        qt_dark_style.applyDarkStyle(self._QApp)

        scene_file = 'view_graph'
        self.graph = load_graph(scene_file.split('.yaml')[0]+'.yaml')

        self._3DView = GLView(self.graph)
        self._pltView = PlotView(self.graph,dx=dx)
        self._GUIView = GUIView(self.sim_run, self.sim_finished_signal, self.sim_progress_signal)
        self._window = self.__init_window(self._3DView, self._pltView, self._GUIView)
 
        self._3DView.setFocusOnObj(self.graph[CAR])

        self.currentStep = 0
        self.play = False

        self._timer = QtCore.QTimer()
        self._timer.timeout.connect(self.update)
        self._timer.start(refresh_rate)

        self.simulator = Simulator(sim_time=20,verbose=False)
        self.graph[GLOBAL_PATH].setData(x=self.simulator.vehicle.path.x,y=self.simulator.vehicle.path.y)

        self.threadpool = QtCore.QThreadPool()


    def __entry_point(self):
        '''
        This entry point is used to trigger the PyQt application
        '''
        # QtGui.QApplication.instance().exec_()
        self._QApp.exec_()

    def __init_window(self,glView,pltView,GUIView):
        '''
        Initialize PyQt window and default camera position.
        '''
        ## Create a GL View widget to display data
        # win = QtGui.QMainWindow()
        win = QtWidgets.QWidget()
        # win.setCentralWidget(area)
        win.resize(WINDOW_WIDTH,WINDOW_HEIGHT)
        win.setWindowTitle(WINDOW_TITLE)

        layout = QtWidgets.QGridLayout(win)
        layout.addWidget(glView,0,0)
        layout.addWidget(GUIView,1,0)
        layout.addWidget(pltView,0,1)
        layout.setColumnMinimumWidth(0,WINDOW_WIDTH/2)
        layout.setColumnMinimumWidth(1,WINDOW_WIDTH/2)
        layout.setRowMinimumHeight(0,WINDOW_HEIGHT/2)
        # layout.setSpacing(2)
 
        # Tried using Dock view from pyqtgraph, but seems to have some bugs when viewing 
        # Will come back to this at a later time.
        # area = DockArea()
        # d1 = Dock('3D View', size=(WINDOW_WIDTH/2, WINDOW_HEIGHT), closable=False)     ## give this dock the minimum possible size
        # d2 = Dock('Plot', size=(WINDOW_WIDTH/2,WINDOW_HEIGHT), closable=False)
        # d3 = Dock('GUI', closable=False)
        # area.addDock(d1, 'left')      ## place d1 at left edge of dock area (it will fill the whole space since there are no other docks yet)
        # area.addDock(d2, 'right')     ## place d2 at right edge of dock area
        # area.addDock(d3,'top',d1)

        # d1.hideTitleBar()
        # d2.hideTitleBar()
        # d3.hideTitleBar()

        # d1.addWidget(glView)
        # d2.addWidget(pltView)
        # d3.addWidget(GUIView)

        win.show()

        return win

    def sim_run(self,finished_call_back=None):
        # Prepare simulation to be run on threads or processes
        worker = Worker(self.sim_run_thread, self.simulator, progress_fn=self.progress_fn, useMultiProcessing=True,verbose=True) 
        worker.signals.result.connect(self.sim_finished)
        worker.signals.progress.connect(self.progress_fn) 
        
        # Execute
        self.threadpool.start(worker) 

    def sim_run_thread(self, sim, q, p):
        log = sim.run(msg=p)
        q.put(log)
        
    def progress_fn(self, n):
        self.sim_progress_signal.emit(n)

    def sim_finished(self,log):
        # print('final x = ' + str(log.x[-1]))
        self.log = log
        self.play = True
        self.currentStep = 0
        self.sim_finished_signal.emit()
    
    def start(self):
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
        if self.play is True:
            if self.__timeReachedEnd():
                self.__resetPlayback()

            self.__updateView()
            self.currentStep += 1
            self._pltView.roll_plot()
    

def main():
    app = Application()
    app.start()


if __name__ == '__main__':
    main()