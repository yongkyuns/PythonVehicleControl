import yaml
from gl_items import Box, Line, Grid, Scatter

def __process_obj(graph,obj,parent=None):
    obj_type = obj['type']
    obj_key = obj['key']
    obj_name = obj['name']
    if 'color' in obj:
        color = obj['color']
    else:
        color = [0,255,0,150] # Default is green, almost opaque
        
    if obj_type == 'Line':
        graph[obj_key] = Line(obj_name,color=color)
    elif obj_type == 'Scatter':
        graph[obj_key] = Scatter(obj_name,color=color)
    elif obj_type == 'Box':
        graph[obj_key] = Box(obj_name,color=color,size=(4,2,1))
    else:
        print('Invalid type specified for object ',obj_name,'in yaml file!!')

    if parent is not None:
        graph[obj_key].setParentItem(parent)

    if 'children' in obj:
        for child_obj in obj['children']:
            parent = graph[obj_key]
            graph = __process_obj(graph,child_obj,parent=parent)

    return graph

def load_graph(file_name):
    with open('./'+file_name) as file:
        yaml_objects = yaml.load(file,Loader=yaml.FullLoader)
        graph = {}
    for obj in yaml_objects:
        graph = __process_obj(graph,obj)
    return graph