'''
.. module:: gl_itmes
   :synopsis: PyQtGraph extension classes for OpenGL objects.
.. moduleauthor:: Yongkyun Shin <github.com/yongkyuns>

This module implements child classes of default PyQtGraph OpenGL items for visualization. 
Some classes (e.g. GLBoxItem) needed to be defined to change their origin (from left bottem corner to center of object) 
for easier manipualtion. Some classes (e.g. lines and scatter) were extended to provide functions for changing
their data in each simulation time step with a common function name to provide consistency.

'''


import pyqtgraph.opengl as gl
from OpenGL.GL import glBegin,glVertex3f,glEnd,glColor4f,glColor,GL_LINES
import numpy as np
from pyqtgraph.Qt import QtCore, QtGui

class Box(gl.GLBoxItem):
    '''
    Box object is a child class of pyqtgraph.opengl.GLBoxItem. The location of the box position is 
    at the origin, which is the x,y center of the vehicle and at 0 height (i.e. center 
    of the bottom surface).
    
    ================  ==================================================
    **Arguments:**
    name              (string) name of the object
    size              ([float,float,float]) length,width,height of the box
    ================  ==================================================
    '''
    def __init__(self, name, size=(1,1,1), color='g', data=(0,0,0,0,0,0)):
        super().__init__(size=QtGui.QVector3D(*size), color=color)
        self.name = name
        self._data = data
        self.setData(data=data)

    def setData(self,x=None,y=None,z=None,x_ang=None,y_ang=None,z_ang=None,data=None):
        '''
        This function is used to change the data of the object for animation. For boxes,
        this function updates the location and orientation. For line and scatter plot, this 
        function updates the plot data. 
        '''
        if data is not None:
            data = list(data) 
            data += [0] * (6-len(data)) # pad any unused values with 0
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

class Car(Box):
    '''
    Class to overwrite the appearnce of Box object to a car shape. 
    Simple cosmetic addition. Not implemented yet.
    '''
    def __init__(self):
        pass
    def paint(self):
        pass

class Line(gl.GLLinePlotItem):
    '''
    Line is a child class of pyqtgraph.opengl.GLLinePlotItem. This class 
    can be used to draw line items (e.g. paths, lanes, etc.) in 3D environment.
    Line.setData function can be used to update the data in each time step.
    
    ================  ==================================================
    **Arguments:**
    name              (string) name of the object
    data              (numpy array) 3-by-n array of (x,y,z) coordinates for plot. 2-by-n also works (assume 2D)
    ================  ==================================================
    '''
    def __init__(self, name, color='g', data=None, **kwargs):
        super().__init__(pos=data, color=color, antialias=True, **kwargs)
        self.name = name
        self._data = data
        # self.setData(data)

    def setData(self, data=None, x=None, y=None, z=None, **kwargs):
        if data is not None:
            super().setData(pos=data,**kwargs)
        elif x is not None and y is not None:
            if z is not None:
                data = np.vstack([x,y,z]).transpose()
            else:
                data = np.vstack([x,y]).transpose()
            self.setData(pos=data)
        else:
            super().setData(**kwargs)

class Scatter(gl.GLScatterPlotItem):
    '''
    Scatter is a child class of pyqtgraph.opengl.GLScatterPlotItem. This class 
    can be used to draw points (e.g. spheres, point-cloud, etc.) in 3D environment.
    Scatter.setData function can be used to update the data in each time step.
    
    ================  ==================================================
    **Arguments:**
    name              (string) name of the object
    size              (float) size of the points
    data              (numpy array) 3-by-n array of (x,y,z) coordinates for plot. 2-by-n also works (assume 2D)
    ================  ==================================================
    '''
    def __init__(self, name, color=[1.0,1.0,0.0,0.5], size=3, data=np.array([0,0,0]), **kwargs):
        super().__init__(pos=data, color=color, size=size, **kwargs)
        self.name = name
        self._data = data
    
    def setData(self, data=None, x=None, y=None, z=None, **kwargs):
        if data is not None:
            super().setData(pos=data,**kwargs)
        elif x is not None and y is not None:
            if z is None:
                z = [0] * len(x)
            data = np.vstack([x,y,z]).transpose()
            self.setData(pos=data)
        else:
            super().setData(**kwargs)

class Grid(gl.GLGridItem):
    '''
    This class adds grid to the 3D view and is a child class of pyqtgraph.opengl.GLGridItem. 
    '''
    def __init__(self, size=(50,50,1), **kwargs):
        super().__init__(size=QtGui.QVector3D(*size),**kwargs)
        self.setDepthValue(10)  # draw grid after surfaces since they may be translucent