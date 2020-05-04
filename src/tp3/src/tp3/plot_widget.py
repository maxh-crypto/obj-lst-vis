import matplotlib.pyplot as plt
import numpy as np

from python_qt_binding import QtCore, QtGui
from python_qt_binding.QtWidgets import QWidget, QLineEdit, QPushButton, QVBoxLayout

from matplotlib.backends.qt_compat import QtCore, QtWidgets, is_pyqt5
if is_pyqt5():
    from matplotlib.backends.backend_qt5agg import (
        FigureCanvas, NavigationToolbar2QT as NavigationToolbar
        )
else:
    from matplotlib.backends.backend_qt4agg import (
        FigureCanvas, NavigationToolbar2QT as NavigationToolbar
        )
from matplotlib.figure import Figure

class PlotWidget(QWidget):
    '''
        This is the widget for the plot area
        besides the matplot it contains a button to add new plots
        and a Slider to adjust the time or the objectID
    '''
    def __init__(self, bagFiles):
        super(PlotWidget, self).__init__()
        self.bagFiles = bagFiles
        
        self.layout = QVBoxLayout()
        
        canvas = FigureCanvas(Figure(figsize=(5, 3)))
        toolbar = NavigationToolbar(canvas, self)
        self.layout.addWidget(toolbar)
        self.layout.addWidget(canvas)
        self.setLayout(self.layout)
        self.ax = canvas.figure.subplots()
        
    def plot(self, plotData):    
        t = plotData[0]
        values = plotData[1]
        self.ax.plot(t, values)
        self.ax._set_label('bag1.obj1.geometric.x')
        self.ax.set_ylabel('value [m]')
        self.ax.set_xlabel('time [s]')
        
        # TODO: Unterscheidung bag1 oder bag2 -> durchgezogen oder dotted
        # TODO: Unterscheidung objid -> Farbe
        # TODO: legende aghaengig von attribute machen
        
        self.ax.grid()
        self.ax.figure.canvas.draw()
        
    
        
        
        
        
        
        
