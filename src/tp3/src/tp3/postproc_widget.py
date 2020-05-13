from multiprocessing import sys

import sys

from python_qt_binding import QtCore, QtGui
from python_qt_binding.QtWidgets import (QWidget, QLabel, QPushButton, QVBoxLayout, QGridLayout)
from bagselector_widget import BagWidget
from plot_widget import PlotWidget
from plotDialog_widget import PlotDialogWidget
from delPlotDialog_widget import DelPlotDialog

class PostProcMainWidget(QWidget):
    '''
       this is the main GUI-Widget for the postprocessing Module
       it inherits from QWidget (Qt) 
       contains three other widgets: BagSelectorWidget, InfoSelectorWidget, PlotWidget
    '''
    def __init__(self):
        super(PostProcMainWidget, self).__init__()

        # init the GUI components:
        self.bagWidget = BagWidget()
        self.bagFiles = self.bagWidget.getBagFiles()
        self.plotWidget = PlotWidget(self.bagFiles)
        self.__addPlotBtn = QPushButton("Add new Graph")
        self.__delPlotBtn = QPushButton("Delete Graph")
        
        # connect signals to slots:
        self.__addPlotBtn.clicked.connect(self.openNewGraphDialog)
        self.__delPlotBtn.clicked.connect(self.openDelGraphDialog)
        
        # setup the layout:
        layout = QGridLayout()
        layout.setRowStretch(0, 0) # controls behavior when changing the window size
        # (widget, fromRow, fromColumn, rowSpan, columnSpan)
        layout.addWidget(self.bagWidget, 0, 0, 1, 3)  
        layout.setRowStretch(1, 0)
        layout.addWidget(self.__addPlotBtn, 1, 0, 1, 1)
        layout.addWidget(self.__delPlotBtn, 1, 1, 1, 1)
        layout.setRowStretch(2, 1)
        layout.addWidget(self.plotWidget, 2, 0, 1, 3)
        
        self.setLayout(layout)
        
    def openNewGraphDialog(self):
        '''
            opens new dialog widget to determine the information required for a new plot
        '''
        self.bagFiles = self.bagWidget.getBagFiles()
        plotDialog = PlotDialogWidget(self.bagFiles)
        plotDialog.newPlotData.connect(self.plotWidget.plot)
        plotDialog.exec_()

    def openDelGraphDialog(self):
        '''
            opens a new dialog to delete one or several graphs from the figure
        '''
        linesList = self.plotWidget.getLines()
        print(linesList)
        delPlotDialog = DelPlotDialog(linesList, self)
        delPlotDialog.deletePressed.connect(self.plotWidget.deleteGraph)
        delPlotDialog.exec_()
        
