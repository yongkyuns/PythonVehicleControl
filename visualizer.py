from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import numpy as np
from OpenGL.GL import glBegin,glVertex3f,glEnd,glColor4f,glColor,GL_LINES

class Box(gl.GLBoxItem):
    def __init__(self, name, size=(1,1,1), color='g', data=(0,0,0,0,0,0)):
        super().__init__(size=QtGui.QVector3D(*size), color=color)
        self.name = name
        self._data = data
        self.setData(data=data)

    def setData(self,x=None,y=None,z=None,x_ang=None,y_ang=None,z_ang=None,data=None):
        if data is not None:
            x,y,z,x_ang,y_ang,z_ang = data
        else:
            if x is None:
                x = self._data[0]
            if y is None:
                y = self._data[1]
            if z is None:
                z = self._data[2]

            if x_ang is None:
                x_ang = self._data[3]
            if y_ang is None:
                y_ang = self._data[4]
            if z_ang is None:
                z_ang = self._data[5]

        self.resetTransform()
        self.translate(x, y, z, local=True)
        if x_ang != self._data[3]:
            self.rotate(x_ang,1,0,0,local=True)
        if y_ang != self._data[4]:
            self.rotate(y_ang,0,1,0,local=True)
        if z_ang != self._data[5]:
            self.rotate(z_ang,0,0,1,local=True)

        self._data = (x,y,z,x_ang,y_ang,z_ang)

    def paint(self):
        '''
        This function overrides GLBoxItem.paint to define the origin with respect to the center of the
        box, instead of using the left lower corner. 
        '''
        self.setupGLState()
        
        glBegin( GL_LINES )
        
        glColor4f(*self.color().glColor())
        x,y,z = self.size()

        glVertex3f(-x/2, -y/2, 0)
        glVertex3f(-x/2, -y/2, z)
        glVertex3f(x/2, -y/2, 0)
        glVertex3f(x/2, -y/2, z)
        glVertex3f(-x/2, y/2, 0)
        glVertex3f(-x/2, y/2, z)
        glVertex3f(x/2, y/2, 0)
        glVertex3f(x/2, y/2, z)

        glVertex3f(-x/2, -y/2, 0)
        glVertex3f(-x/2, y/2, 0)
        glVertex3f(x/2, -y/2, 0)
        glVertex3f(x/2, y/2, 0)
        glVertex3f(-x/2, -y/2, z)
        glVertex3f(-x/2, y/2, z)
        glVertex3f(x/2, -y/2, z)
        glVertex3f(x/2, y/2, z)
        
        glVertex3f(-x/2, -y/2, 0)
        glVertex3f(x/2, -y/2, 0)
        glVertex3f(-x/2, y/2, 0)
        glVertex3f(x/2, y/2, 0)
        glVertex3f(-x/2, -y/2, z)
        glVertex3f(x/2, -y/2, z)
        glVertex3f(-x/2, y/2, z)
        glVertex3f(x/2, y/2, z)
        
        glEnd()

class Line(gl.GLLinePlotItem):
    def __init__(self, name, color='g', data=None, **kwargs):
        super().__init__(pos=data, color=color, antialias=True, **kwargs)
        self.name = name
        self._data = data
        # self.setData(data)

    def setData(self, data=None, **kwargs): 
        if data is not None:
            super().setData(pos=data,**kwargs)
        else:
            super().setData(**kwargs)

class Scatter(gl.GLScatterPlotItem):
    def __init__(self, name, color=[1.0,1.0,0.0,0.5], size=13, data=np.array([0,0]), **kwargs):
        super().__init__(pos=data, color=color, size=size, **kwargs)
        self.name = name
        self._data = data
    
    def setData(self, data=None, **kwargs):
        if data is not None:
            super().setData(pos=data,**kwargs)
        else:
            super().setData(**kwargs)

class Grid(gl.GLGridItem):
    def __init__(self, size=(50,50,1), **kwargs):
        super().__init__(size=QtGui.QVector3D(*size),**kwargs)
        self.setDepthValue(10)  # draw grid after surfaces since they may be translucent
    

class Visualizer():
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
        QtGui.QApplication.instance().exec_()

    def init_window(self,name):
        ## Create a GL View widget to display data
        app = QtGui.QApplication([])
        w = gl.GLViewWidget() 
        w.resize(1900/2,1080/2)
        w.show()
        w.setWindowTitle(name)
        w.setCameraPosition(distance=50,elevation=90,azimuth=270) #elevation of 90deg is top view, azimuth=top-view-rotation
        return app, w
    
    def add_grid(self):
        ## Add a grid to the view
        g = Grid()
        g.scale(5,5,5)
        self._window.addItem(g)
    
    def update(self):
        self.update_func()
        self.update_origin()
    
    def update_origin(self):
        x,y = self.car.mapToView((0,0))
        cam_center = self._window.opts['center']
        self._window.opts['center'] = QtGui.QVector3D(x, y, cam_center[2])



def update_func():
    return 0, 0, 0

def main():
    view = Visualizer(update_func)
    view.entry_point()


if __name__ == '__main__':
    main()