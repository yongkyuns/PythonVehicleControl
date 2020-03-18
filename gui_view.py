
from pyqtgraph.widgets.LayoutWidget import LayoutWidget
from pyqtgraph.Qt import QtGui

from multiprocessing import Process, Queue

class GUIView(LayoutWidget):

    def __init__(self, run, run_finished, run_progress):
        super().__init__()
        # w = LayoutWidget()
        self.run_func = run
        self.runButton = QtGui.QPushButton('Run')
        self.addWidget(self.runButton, row=0, col=0)

        self.runButton.clicked.connect(self.runButtonClicked)
        run_finished.connect(self.runFinished)

        self.progres_bar = QtGui.QProgressBar()
        self.addWidget(self.progres_bar, row=1, col=0)
        run_progress.connect(self.updateProgress)

        # Configure callback connection
        
    
    def runButtonClicked(self):
        self.runButton.setEnabled(False)
        self.progres_bar.setValue(0)
        self.run_func(finished_call_back=self.runFinished)
        # self.runButton.setEnabled(True)

    def runFinished(self):
        self.runButton.setEnabled(True)
        self.progres_bar.setValue(0)

    def updateProgress(self,val):
        self.progres_bar.setValue(val)

