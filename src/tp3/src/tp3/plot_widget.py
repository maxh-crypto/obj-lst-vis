import matplotlib.pyplot as plt
import numpy as np

from python_qt_binding import QtCore, QtGui
from python_qt_binding.QtWidgets import QWidget, QLineEdit, QPushButton, QVBoxLayout
from plotDialog_widget import PlotDialogWidget

from matplotlib.backends.qt_compat import QtCore, QtWidgets, is_pyqt5
if is_pyqt5():
    from matplotlib.backends.backend_qt5agg import (
        FigureCanvas
        #, NavigationToolbar2QT as NavigationToolbar
        )
else:
    from matplotlib.backends.backend_qt4agg import (
        FigureCanvas
        #, NavigationToolbar2QT as NavigationToolbar
        )
from matplotlib.figure import Figure

class PlotWidget(QWidget):
    '''
        This is the widget for the plot area
        besides the matplot it contains a button to add new plots
        and a Slider to adjust the time or the objectID
    '''
    def __init__(self):
        super(PlotWidget, self).__init__()
        self.layout = QVBoxLayout()
        self.__addPlotBtn = QPushButton("Add new Plot")
        self.__addPlotBtn.clicked.connect(self.openDialog)
        self.layout.addWidget(self.__addPlotBtn)
        
        canvas = FigureCanvas(Figure(figsize=(5, 3)))
        self.layout.addWidget(canvas)
        self.setLayout(self.layout)
        self.ax = canvas.figure.subplots()
        self.plot()
        
#     def plot(self):    
#         t = np.linspace(0, 10, 101)
#         self.ax.set_ylim(-1.1, 1.1)
#         self.ax.plot(t, np.sin(t))
#         self.ax.figure.canvas.draw()
        
    def openDialog(self):
        '''
            opens new dialog widget to determine the information required for a new plot
        '''
        plotDialog = PlotDialogWidget()
        plotDialog.exec_()
        
        
        
        
        
        
