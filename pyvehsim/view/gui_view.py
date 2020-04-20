
from pyqtgraph.widgets.LayoutWidget import LayoutWidget
from pyqtgraph.Qt import QtGui, QtWidgets, QtCore

from multiprocessing import Process, Queue

class GUIView(LayoutWidget):

    sim_time_changed_signal = QtCore.pyqtSignal(int)

    def __init__(self, run_func, run_finished_signal, run_progress_signal, init_sim_time):
        super().__init__()
        # w = LayoutWidget()

        self.run_func = run_func
        self.runButton = QtGui.QPushButton('Run')
        # self.addWidget(self.runButton, row=0, col=1)

        self.runButton.clicked.connect(self.runButtonClicked)
        run_finished_signal.connect(self.runFinished)

        sim_time_label = QtWidgets.QLabel()
        sim_time_label.setText('Simulation Time:')
        self.addWidget(sim_time_label,row=0,col=0)

        self.sim_time_edit = QtWidgets.QLineEdit(str(init_sim_time))
        self.sim_time_edit.setValidator(QtGui.QIntValidator())
        self.addWidget(self.sim_time_edit,row=0, col=1)
        # self.sim_time_edit.returnPressed.connect(self.sim_time_changed)
        self.sim_time_edit.textChanged.connect(self.sim_time_changed)

        self.progres_bar = QtGui.QProgressBar()
        # self.addWidget(self.progres_bar, row=2, col=1)
        run_progress_signal.connect(self.updateProgress)

        self.runButtonStack = QtWidgets.QStackedWidget()
        self.runButtonStack.addWidget(self.runButton)
        self.runButtonStack.addWidget(self.progres_bar)
        self.runButtonStack.setCurrentIndex(0)
        self.addWidget(self.runButtonStack,row=1,col=1)


    def runButtonClicked(self):
        self.runButton.setEnabled(False)
        self.progres_bar.setValue(0)
        self.runButtonStack.setCurrentIndex(1)
        self.run_func(finished_call_back=self.runFinished)
        # self.runButton.setEnabled(True)

    def runFinished(self):
        self.runButton.setEnabled(True)
        self.runButtonStack.setCurrentIndex(0)
        self.progres_bar.setValue(0)

    def updateProgress(self,val):
        self.progres_bar.setValue(val)

    def sim_time_changed(self):
        # print(self.sim_time_edit.text())
        if len(self.sim_time_edit.text()) > 0:
            self.sim_time_changed_signal.emit(int(self.sim_time_edit.text()))

