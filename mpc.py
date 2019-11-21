import scipy


class Params:
    def __init__(self,Vx,m,Iz,lf,lr,Caf,Car):
        self._callbacks = []

        self.Vx = Vx
        self.m = m
        self.Iz = Iz
        self.lf = lf
        self.lr = lr
        self.Caf = Caf
        self.Car = Car

    def register_callback(self, callback):
        self._callbacks.append(callback)
    
    def param_changed(self, *args, **kwargs):
        for callback in self._callbacks:
            callback(self, *args, **kwargs)

    @property
    def Vx(self):
        return self._Vx
    @Vx.setter
    def Vx(self,value):
        self._Vx = value
        self.param_changed()
    
    @property
    def m(self):
        return self._m
    @m.setter
    def m(self,value):
        self._m = value
        self.param_changed()

    @property
    def Iz(self):
        return self._Iz
    @Iz.setter
    def Iz(self,value):
        self._Iz = value
        self.param_changed()

    @property
    def lf(self):
        return self._lf
    @lf.setter
    def lf(self,value):
        self._lf = value
        self.param_changed()

    @property
    def lr(self):
        return self._lr
    @lr.setter
    def lr(self,value):
        self._lr = value
        self.param_changed()

    @property
    def Caf(self):
        return self._Caf
    @Caf.setter
    def Caf(self,value):
        self._Caf = value
        self.param_changed()

    @property
    def Car(self):
        return self._Car
    @Car.setter
    def Car(self,value):
        self._Car = value
        self.param_changed()

class VehicleModel:
    def __init__(self,A,B,C,D):
        self.A = A
        self.B = B
        self.C = C
        self.D = D

class Vehicle:

    def __init__(self,Vx=60,m=1600,Iz=1705,lf=1.0,lr=1.0,Caf=66243,Car=66243):
        self.params = Params(Vx,m,Iz,lf,lr,Caf,Car)
        self.params.register_callback(self.update)
        self.__update_model()

    @property
    def params(self):
        return self._params
    @params.setter
    def params(self,value):
        self._params = value
        self.__update_model()
    
    @property
    def model(self):
        return self._model
    @model.setter
    def model(self,value):
        self._model = value
    
    def update(self, observable, *args, **kwargs):
        self.__update_model()

    def __update_model(self):
        p = self.params
        Vx,m,Iz,lf,lr,Caf,Car = p.Vx, p.m, p.Iz, p.lf, p.lr, p.Caf, p.Car

        A = [[-(2*Caf + 2*Car)/(m*Vx),       0,  -Vx-(2*Caf*lf-2*Car*lr)/(m*Vx),     0],
             [        0,                     0,              1,                      0],
             [-(2*lf*Caf-2*lr*Car)/(Iz*Vx),  0,  -(2*lf**2*Caf+2*lr**2*Car)/(Iz*Vx), 0],
             [        1,                     Vx,             0,                      0]]

        B = [[  2*Caf/m  ],
             [     0     ],
             [2*lf*Caf/Iz],
             [     0     ]]

        C = [[0, 0, 0, 1],
             [0, 1, 0, 0]]

        D = 0

        print('Vehicle parameters changed... Updating model...')
        self.model = VehicleModel(A,B,C,D)


car = Vehicle()

