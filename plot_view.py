from plot_items import Signal
from pyqtgraph.widgets.GraphicsLayoutWidget import GraphicsLayoutWidget
from pyqtgraph.graphicsItems.PlotItem import PlotItem

class PlotView(GraphicsLayoutWidget):
    def __init__(self,obj_dict,dx=0.01):
        super().__init__(show=True)

        plot = PlotItem()
        plot.setDownsampling(mode='peak')
        plot.setClipToView(True)
        plot.showGrid(x=True,y=True)
        plot.addLegend()
        plot.disableAutoRange()

        self.addItem(plot,row=1,col=1)

        self.dx = dx # dx is used to roll the plots in time for each time step
        self.auto_roll_plot = True

        for _,obj in obj_dict.items():
            if self.is_plot_obj(obj):
                plot = self.getItem(1,1)
                plot.addItem(obj)
    
    def is_plot_obj(self,obj):
        if isinstance(obj,Signal):
            return True
        else:
            return False

    def roll_plot(self):
        plot = self.getItem(row=1,col=1)
        viewBox = plot.getViewBox()
        viewBox.translateBy(x=self.dx)

    def translateBy(self,dx):
        plot = self.getItem(row=1,col=1)
        viewBox = plot.getViewBox()
        viewBox.translateBy(x=dx)
