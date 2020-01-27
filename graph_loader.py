import yaml
from gl_items import Box, Line, Grid, Scatter
from plot_items import Signal
import pyqtgraph as pg

# Names of the dictionary keys used in yaml graph file
TYPE = 'type'
KEY = 'key'
NAME = 'name'
COLOR = 'color'
CHILDREN = 'children'

# Class names supported for visualization
LINE = 'Line'
SCATTER = 'Scatter'
BOX = 'Box'
SIGNAL = 'Signal'

def __process_obj(graph,obj,parent=None):
    obj_type = obj[TYPE]
    obj_key = obj[KEY]
    obj_name = obj[NAME]
    if COLOR in obj:
        color = obj[COLOR]
    else:
        color = [0,255,0,150] # Default is green, almost opaque

    if obj_type == 'Line':
        graph[obj_key] = Line(obj_name,color=color)
    elif obj_type == 'Scatter':
        graph[obj_key] = Scatter(obj_name,color=color)
    elif obj_type == 'Box':
        graph[obj_key] = Box(obj_name,color=color,size=(4,2,1))
    elif obj_type == 'Signal':
        graph[obj_key] = Signal(obj_name,color=color)
        # pen = pg.mkPen(color=color, width=1.5, style=QtCore.Qt.SolidLine)
        # graph[obj_key] = pg.graphicsItems.PlotDataItem.PlotDataItem(name=obj_name,pen=pen)
    else:
        print('Invalid type specified for object ',obj_name,'in yaml file!!')

    if parent is not None:
        graph[obj_key].setParentItem(parent)

    if CHILDREN in obj:
        for child_obj in obj[CHILDREN]:
            parent = graph[obj_key]
            graph = __process_obj(graph,child_obj,parent=parent)

    return graph

def is_plot_obj(obj):
    '''
    Return True if the object passed is a 2D Signal object, otherwise return False
    '''
    if obj[TYPE] == SIGNAL:
        return True
    else:
        return False

def load_graph(file_name):
    with open('./'+file_name) as file:
        yaml_objects = yaml.load(file,Loader=yaml.FullLoader)
        graph = {}
    for obj in yaml_objects:
        graph = __process_obj(graph,obj)
    return graph