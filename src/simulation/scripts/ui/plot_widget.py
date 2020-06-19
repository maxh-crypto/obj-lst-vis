'''
    this widget is the main plot area with the toolbar and the legend area
    the plot area is a matplotlib figure
'''

import matplotlib
import matplotlib.pyplot as plt
import numpy as np

from python_qt_binding import QtCore, QtGui
from python_qt_binding.QtWidgets import (QWidget, QLineEdit, QPushButton, 
                                         QVBoxLayout, QHBoxLayout)

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
from legendMeanTable_widget import LegendInfoTable

MAX_LINES = 3

class PlotWidget(QWidget):
    '''
        This is the widget for the plot area
        besides the matplot it contains a button to add new plots
    '''
    
    lineCount = 0 # counter for all active axes 
    linesList = []
    
    def __init__(self, bagFiles, parent=None):
        super(PlotWidget, self).__init__()
        self.bagFiles = bagFiles
        self.parent = parent
        
        # init the components:
        self.canvas = FigureCanvas(Figure(figsize=(5, 3)))
        toolbar = NavigationToolbar(self.canvas, self)
        self.legendInfoWidget = LegendInfoTable(self)
        self.legendInfoWidget.setMaximumWidth(600)
        self.legendInfoWidget.setMaximumHeight(110)

        self.ax = self.canvas.figure.subplots()
        
        # layout
        figLayout = QVBoxLayout()
        figLayout.addWidget(toolbar)
        figLayout.addWidget(self.canvas)
        figLayout.addWidget(self.legendInfoWidget)
        self.setLayout(figLayout)
        
        
    def plot(self, plotData, plotInfo):  
        '''
            is connected to the clicked signal of the dialog start button
            plots the given data into the figure
        '''
        t = plotData[0]
        values = plotData[1]            
        
        line, = self.ax.plot(t, values, 'x')
        
        label = plotInfo['label']
        line.set_label(label)

        self.ax.set_xlabel('time [ms]')
        
        self.ax.grid(b=True)
        self.ax.figure.canvas.draw()
        
        self.linesList.append(line)
        
        # get the line color
        color_rgba = matplotlib.colors.to_rgba(line.get_color())
        
        # add a new row to the legend
        self.legendInfoWidget.addRow(label, color_rgba, plotData[2], plotData[3])
        self.lineCount += 1
        
        
    def deleteGraph(self, lineNr):
        '''
            deletes the specified line in the figure
        '''
        line = self.linesList[lineNr]
        self.linesList.remove(line)
        self.ax.lines.remove(line)
        del(line)
        
        # remove this specific line from the legend
        self.legendInfoWidget.removeRow(lineNr)
        
        self.lineCount -= 1
        self.ax.figure.canvas.draw()
        
        
    def getLines(self):
        '''
            returns a list of all lines in the figure
        '''
        return self.linesList
        
        
        
        
        
