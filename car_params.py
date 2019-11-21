class CarParams:
    """
    Collection of car parameters to define dynamics model.
    Supports callbacks from owner class when attributes change.
    Currently supported parameters define bicycle model.
    """
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

