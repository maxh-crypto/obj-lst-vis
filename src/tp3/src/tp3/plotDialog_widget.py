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

class PlotDialogWidget(QDialog):    
    newPlotData = QtCore.Signal(object)
    
    def __init__(self, bagFiles, parent=None):
        super(PlotDialogWidget, self).__init__()
        self.parent = parent
        self.bagFiles = bagFiles
        self.setWindowTitle("Add new Plot")
        # self.setWindowModality(Qt.ApplicationModal)
        self.layout = QVBoxLayout()        
        self.resize(600, 400)
        
        # TabWidget
        self.tabWidget = QTabWidget()
        self.rawDataTab = RawDataTab(bagFiles)
        self.compareTab = CompareDataTab() 
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
        if currentTab == 0: # rawDataTab is active
            self.getRawData()
        if currentTab == 1: # 
            # TODO
            pass
            
        
    def getRawData(self):
        '''
            gets the raw data according to the selected parameters
            from Rosbag_Analysis
        '''
        selectedValue = self.rawDataTab.valueWidget.getCatAndAtt()
        category = selectedValue['category']
        attribute = selectedValue['attribute']   
        # check whether category or attribute is empty
        # show error message when it is the case
        # and return the function
        if category == "" or attribute == "":
            self.showMessage("Please select a plottable attribute.")
            return 
        
        if self.rawDataTab.selectedBag > 1: # no bag file is selected
            self.showMessage("Please select a bag file.")
            return
            
        bagfile = self.bagFiles[self.rawDataTab.selectedBag]
        if bagfile == "":
            self.showMessage("no bag file loaded! Please import bag file in the main interface.")
            return
        # bagfile = "/home/max/obj-lst-vis/src/tp3/bagfiles/2020-04-24-18-36-14.bag"
        
        try:
            obj_id = self.rawDataTab.idSelector.getID()
        except ValueError:
            self.showMessage("ObjectID is not a number! Insert valid ID.")
            return
        try:    
            plotData = Rosbag_Analysis.getRawData(bagfile, obj_id, category, attribute)
        except:
            self.showMessage("Sorry, unexpected error occurred.")
            return
            
	    # emit signal with data
        self.newPlotData.emit(plotData)
        
        # close dialog
        self.close()
        
    def showMessage(self, msgText):
        msgBox = QMessageBox()
        msgBox.setText(msgText)
        msgBox.exec_()
        
        

