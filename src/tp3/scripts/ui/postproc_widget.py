import sys
sys.path.append("../")
import message_module

from python_qt_binding import QtCore, QtGui
from python_qt_binding.QtWidgets import (QWidget, QLabel, QPushButton, QVBoxLayout, QGridLayout)
from bagselector_widget import BagWidget
from plot_widget import PlotWidget, MAX_LINES
from plotDialog_widget import PlotDialogWidget
from delPlotDialog_widget import DelPlotDialog
from QualityDialog_widget import QualityDialog
from legendMeanTable_widget import LegendInfoTable

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
        self.legendInfoWidget = LegendInfoTable(self)
        self.__addPlotBtn = QPushButton("Add new Graph")
        self.__delPlotBtn = QPushButton("Delete Graph")
        self.__qualityBtn = QPushButton("Compute Data Quality")
        
        # connect signals to slots:
        self.__addPlotBtn.clicked.connect(self.openNewGraphDialog)
        self.__delPlotBtn.clicked.connect(self.openDelGraphDialog)
        self.__qualityBtn.clicked.connect(self.openQualityDialog)
        
        # setup the layout:
        layout = QGridLayout()
        # (rowIndex, rowStretch)
        layout.setRowStretch(0, 0) # controls behavior when changing the window size
        layout.setColumnStretch(0, 1)
        layout.setColumnStretch(1, 1)
        layout.setColumnStretch(2, 1)
        # (widget, fromRow, fromColumn, rowSpan, columnSpan)
        layout.addWidget(self.bagWidget, 0, 0, 1, 3)  
        layout.setRowStretch(1, 0)
        
        layout.addWidget(self.__addPlotBtn, 1, 0, 1, 1)
        layout.addWidget(self.__delPlotBtn, 1, 1, 1, 1)
        layout.addWidget(self.__qualityBtn, 1, 2, 1, 1)
        
        layout.setRowStretch(2, 1)
        layout.addWidget(self.plotWidget, 2, 0, 2, 2)
        layout.addWidget(self.legendInfoWidget, 3, 2, 1, 1)
        
        self.setLayout(layout)
        
    def openNewGraphDialog(self):
        '''
            opens new dialog widget to determine the information required for a new plot
        '''
        if self.plotWidget.lineCount >= MAX_LINES:
            message_module.showMessage("Maximum number of graphs reached. Delete a graph before adding a new one.")
            return 
        
        self.bagFiles = self.bagWidget.getBagFiles()
        plotDialog = PlotDialogWidget(self.bagFiles, self)
        plotDialog.newPlotData.connect(self.plotWidget.plot)
        plotDialog.exec_()
        

    def openDelGraphDialog(self):
        '''
            opens a new dialog to delete one or several graphs from the figure
        '''
        linesList = self.plotWidget.getLines()
        delPlotDialog = DelPlotDialog(linesList, self)
        delPlotDialog.deletePressed.connect(self.plotWidget.deleteGraph)
        delPlotDialog.exec_()
        
    
    def openQualityDialog(self):
        '''
            opens a new dialog that shows the data quality
        '''
        self.bagFiles = self.bagWidget.getBagFiles()
        
        if self.bagFiles[0] == "" or self.bagFiles[1] == "":
            message_module.showMessage("Bag file missing! Please import bag file in the main interface.")  
            return
        
        qualityDialog = QualityDialog(self.bagFiles, self)
        qualityDialog.exec_()
