
class Logger():
    def __init__(self):
        self.out = []
        self.ref = []
        self.x = []
        self.y = []
        self.yaw = []
        self.str_ang = []
        self.path_x = []
        self.path_y = []
        self.ctrl_pt_x = []
        self.ctrl_pt_y = []
    
    def log(self, ctrl_pt_x, ctrl_pt_y, path_x, path_y, out, ref, x, y, yaw, str_ang):
        self.out.append(out)
        self.ref.append(ref)
        self.x.append(x)
        self.y.append(y)
        self.yaw.append(yaw)
        self.str_ang.append(str_ang)
        self.path_x.append(path_x)
        self.path_y.append(path_y)
        self.ctrl_pt_x.append(ctrl_pt_x)
        self.ctrl_pt_y.append(ctrl_pt_y)