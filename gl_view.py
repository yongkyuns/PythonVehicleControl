# import pyqtgraph.opengl as gl
from pyqtgraph.opengl.GLViewWidget import GLViewWidget
from gl_items import Box, Line, Grid, Scatter
from pyqtgraph.Qt import QtGui

class GLView(GLViewWidget):
    def __init__(self, obj_dict):
        super().__init__()
        self.add_grid()
        self.setCameraPosition(distance=50,elevation=90,azimuth=270) #elevation of 90 deg is top view, azimuth=top-view-rotation

        self._focusObj = None
        self.focus_on = True

        for _,obj in obj_dict.items():
            if self.is_gl_obj(obj):
                self.addItem(obj)
    
    def is_gl_obj(self,obj):
        if isinstance(obj,Box) or isinstance(obj,Line) or isinstance(obj,Scatter) or isinstance(obj,Grid):
            return True
        else:
            return False

    def add_grid(self):
        '''
        Add grid to the view
        '''
        g = Grid()
        g.scale(5,5,5)
        self.addItem(g)

    def setFocusOnObj(self,obj):
        self._focusObj = obj

    def update_origin(self):
        '''
        Update the camaera focus point
        '''
        if self._focusObj is not None:
            x,y = self._focusObj.mapToView((0,0))
            cam_center = self.opts['center']
            self.opts['center'] = QtGui.QVector3D(x, y, cam_center[2])
    
    def paintGL(self, region=None, viewport=None, useItemNames=False):
        if self.focus_on:
            self.update_origin()
        super().paintGL(region=region, viewport=viewport, useItemNames=useItemNames)
