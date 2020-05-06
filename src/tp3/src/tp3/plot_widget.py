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
        # print(plotData)
        t = plotData[0]
        values = plotData[1]
        line, = self.ax.plot(t, values)
        
        label = 'bag' + str(plotInfo['bagFile_id']) + '.'
        label += 'obj' + str(plotInfo['obj_id']) + '.'
        label += plotInfo['category'] + '.'
        label += plotInfo['attribute']
        line.set_label(label)
        unit = units[plotInfo['attribute']]
        self.ax.set_ylabel('value [' + unit + ']')
        self.ax.set_xlabel('time [ms]')
        self.ax.legend()
#         self.canvas.figure.legend(loc='upper center', bbox_to_anchor=(1.45, 0.8), shadow=True, ncol=1)
        
        # TODO: Unterscheidung bag1 oder bag2 -> durchgezogen oder dotted
        # TODO: Unterscheidung objid -> Farbe
        # TODO: legende aghaengig von attribute machen
        
        self.ax.grid(b=True)
        self.ax.figure.canvas.draw()
        
    
        
        
        
        
        
        
