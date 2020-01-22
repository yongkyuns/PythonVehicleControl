import yaml
from gl_items import Box, Line, Grid, Scatter
from plot_items import Plot

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
PLOT = 'Plot'

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
    elif obj_type == 'Plot':
        graph[obj_key] = Plot(obj_name,color=color)
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
    Return True if the object passed is a 2D Plot object, otherwise return False
    '''
    if obj[TYPE] == PLOT:
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