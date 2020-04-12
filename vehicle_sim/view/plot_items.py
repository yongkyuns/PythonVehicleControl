'''
.. module:: plot_itmes
   :synopsis: PyQtGraph extension classes for plot widget objects.
.. moduleauthor:: Yongkyun Shin <github.com/yongkyuns>

This module implements child classes of default PyQtGraph plot items for 2D graph visualization. 

'''

from pyqtgraph.graphicsItems.PlotItem import PlotItem
from pyqtgraph.graphicsItems.PlotDataItem import PlotDataItem
import numpy as np
from pyqtgraph.functions import mkPen
from pyqtgraph.Qt import QtCore

class Plot(PlotItem):
    '''
    Plot is a child class of pyqtgraph.graphicsItems.PlotItem. This class 
    can be used to draw 2D graph plots. This class is currently just a wrapper 
    that renames PlotItem.
    
    ================  ==================================================
    **Arguments:**
    name              (string) name of the object
    data              (numpy array) 3-by-n array of (x,y,z) coordinates for plot. 2-by-n also works (assume 2D)
    ================  ==================================================
    '''

    def __init__(self, name, color='g', data=None, **kwargs):
        super().__init__(color=color, name=name, **kwargs)
        self._data = data

        self.pen = mkPen(color=color, width=1.5, style=QtCore.Qt.SolidLine)
        self.plotDataItem = PlotDataItem(pen=self.pen,antialias=False,name=name)
        self.addItem(self.plotDataItem)

    def setData(self, data=None, **kwargs): 
        if data is not None:
            self.plotDataItem.setData(data)
            self._data = data
        else:
            self.plotDataItem.setData(**kwargs)
            # Update self._data here

        # if data is not None:
        #     dataType = dataType(data)
        #     if dataType == 'Nx2array':
        #         super().setData(data.tranpose(),**kwargs)
        #     elif dataType == 
        #     else:       
        # else:
        #     print('Data is None!!')
            
class Signal(PlotDataItem):

    def __init__(self, name, color='g', data=None, **kwargs):
        self.pen = mkPen(color=color, width=1.5, style=QtCore.Qt.SolidLine)
        self._data = data
        super().__init__(pen=self.pen, antialias=False, name=name, **kwargs)

    def setData(self, data=None, **kwargs): 
        if data is not None:
            super().setData(data)
            self._data = data
        else:
            super().setData(**kwargs)
            # Update self._data here