import qdarkstyle
import os

os.environ['PYQTGRAPH_QT_LIB'] = 'PyQt5'

def applyDarkStyle(app):
    app.setStyleSheet(qdarkstyle.load_stylesheet(qt_api=os.environ['PYQTGRAPH_QT_LIB']))