# import matplotlib
# matplotlib.rcParams['text.usetex'] = True
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
from object_list_msg import units

MAX_LINES = 3

class PlotWidget(QWidget):
    '''
        This is the widget for the plot area
        besides the matplot it contains a button to add new plots
    '''
    
    lineCount = 0 # counter for all active axes 
    linesList = []
    
    def __init__(self, bagFiles):
        super(PlotWidget, self).__init__()
        self.bagFiles = bagFiles
        
        self.layout = QVBoxLayout()
        
        self.canvas = FigureCanvas(Figure(figsize=(5, 3)))
        toolbar = NavigationToolbar(self.canvas, self)
        self.layout.addWidget(toolbar)
        self.layout.addWidget(self.canvas)
        self.setLayout(self.layout)
        self.ax = self.canvas.figure.subplots()
        
    def plot(self, plotData, plotInfo):  
        '''
            is connected to the clicked signal of the dialog start button
            plots the given data into the figure
        '''
        t = plotData[0]
        values = plotData[1]
        line, = self.ax.plot(t, values, '.')
        
        line.set_label(plotInfo['label'])
        self.ax.set_ylabel(plotInfo['y_label'])
        self.ax.set_xlabel('time [ms]')
        if plotInfo['bag'] == 2:
            line.set_linestyle('dashed')
        
        self.ax.legend()
        
        # TODO: Unterscheidung bag1 oder bag2 -> durchgezogen oder dotted
        # TODO: Unterscheidung objid -> Farbe
        # TODO: mehrere y-Achsen (axis)
        
        self.ax.grid(b=True)
        self.ax.figure.canvas.draw()
        
        self.lineCount += 1
        self.linesList.append(line)
        
    def deleteGraph(self, lineNr):
        line = self.linesList.pop(lineNr)
        line.remove()
        self.lineCount -= 1
        self.ax.figure.canvas.draw()
        self.ax.legend()
        
    def getLines(self):
        '''
            returns a list of all lines in the figure
        '''
        return self.linesList
        
        
        
        
        
