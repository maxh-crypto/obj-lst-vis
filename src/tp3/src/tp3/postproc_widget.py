from multiprocessing import sys

import sys

from python_qt_binding import QtCore, QtGui
from python_qt_binding.QtWidgets import (QWidget, QLabel, QPushButton, QVBoxLayout, QGridLayout)
from bagselector_widget import BagWidget
from plot_widget import PlotWidget
from plotDialog_widget import PlotDialogWidget

class PostProcMainWidget(QWidget):
    '''
       this is the main GUI-Widget for the postprocessing Module
       it inherits from QWidget (Qt) 
       contains three other widgets: BagSelectorWidget, InfoSelectorWidget, PlotWidget
    '''
    def __init__(self):
        super(PostProcMainWidget, self).__init__()

        self.bagWidget = BagWidget()
        self.bagFiles = self.bagWidget.getBagFiles()
        self.plotWidget = PlotWidget(self.bagFiles)
        self.__addPlotBtn = QPushButton("Add new Plot")
        self.__addPlotBtn.clicked.connect(self.openDialog)

        layout = QGridLayout()
        layout.setRowStretch(0, 0)
        # (widget, fromRow, fromColumn, rowSpan, columnSpan)
        layout.addWidget(self.bagWidget, 0, 0, 1, 3)  
        layout.setRowStretch(1, 0)
        layout.addWidget(self.__addPlotBtn, 1, 2, 1, 1)
        layout.setRowStretch(2, 1)
        layout.addWidget(self.plotWidget, 2, 0, 1, 3)
        self.setLayout(layout)
        
    def openDialog(self):
        '''
            opens new dialog widget to determine the information required for a new plot
        '''
        self.bagFiles = self.bagWidget.getBagFiles()
        plotDialog = PlotDialogWidget(self.bagFiles)
        plotDialog.newPlotData.connect(self.plotWidget.plot)
        plotDialog.exec_()

        
        


