'''
    dialog widget for selecting the information 
    that should be presented in a new plot
'''

import sys

from python_qt_binding import QtCore, QtGui
from python_qt_binding.QtWidgets import QDialog, QTabWidget, QVBoxLayout, QPushButton, QMessageBox
from rawdata_tab import RawDataTab
from compareData_tab import CompareDataTab
from Rosbag_Analysis import Rosbag_Analysis

import message_module

class PlotDialogWidget(QDialog):    
    newPlotData = QtCore.Signal(object, object)
    
    def __init__(self, bagFiles, parent=None):
        super(PlotDialogWidget, self).__init__()
        self.parent = parent
        self.bagFiles = bagFiles
        self.setWindowTitle("Add new Graph")
        # self.setWindowModality(Qt.ApplicationModal)
        self.layout = QVBoxLayout()        
        self.resize(600, 400)
        
        # TabWidget
        self.tabWidget = QTabWidget()
        self.rawDataTab = RawDataTab(bagFiles, self)
        self.compareTab = CompareDataTab(bagFiles, self) 
        self.tabWidget.addTab(self.rawDataTab, "Raw Data")
        self.tabWidget.addTab(self.compareTab, "Compare Bag Files")
        self.layout.addWidget(self.tabWidget)
        
        # init the start button
        startBtn = QPushButton("Start")
        startBtn.clicked.connect(self.startPressed)
        self.layout.addWidget(startBtn)
        
        self.setLayout(self.layout)
        
    def startPressed(self):
        '''
            is called when the start button is clicked
            calls the function to get the data to plot
            dependent on what tab is selected            
        '''        
        currentTab = self.tabWidget.currentIndex()
        try:
            
            if currentTab == 0: # rawDataTab is active
                plotData, plotInfo = self.rawDataTab.getPlotData()
            elif currentTab == 1: # 
                plotData, plotInfo = self.compareTab.getPlotData()
                
            # emit signal with data
            self.newPlotData.emit(plotData, plotInfo)
            
            # close dialog 
            self.close()
            
        except Exception as e:
            message_module.showMessage(str(e))
        
