'''
    this widget inherits from QWidget and 
    contains widgets to select the data required 
    for evaluating the camera data and plotting the results
'''

from python_qt_binding import QtCore, QtGui
from python_qt_binding.QtWidgets import (QWidget, QHBoxLayout)
from operationSelector_box import OperationSelectorWidget
from Rosbag_Analysis import Rosbag_Analysis
from ThresholdSetter_box import ThresholdSetter

import object_list_msg

class CompareDataTab(QWidget):
    
    def __init__(self, bagFiles, parent=None):
        super(CompareDataTab, self).__init__()
        self.parent = parent
        
        # attributes
        self.bagFiles = bagFiles        
        
        # widgets
        self.operationSelector = OperationSelectorWidget(self)
        self.thresholdSetter = ThresholdSetter(self)
        
        # layout:
        layout = QHBoxLayout()
        layout.addWidget(self.operationSelector)
        layout.addWidget(self.thresholdSetter)
        
        self.setLayout(layout)
            
            
    def getPlotData(self):
        '''
            returns plotData and plotInfo 
        '''
        plotInfo = {
            'label' : '',
            'y_label' : '',
            }
        
        if self.bagFiles[0] == '' or self.bagFiles[1] == '':
            raise Exception("Bag file missing. Please import two bag files in the main interface.")
        
        threshold = self.thresholdSetter.getThreshold()
        
        # get operation
        operation = self.operationSelector.getOperation()
        
        if operation == '':
            raise Exception("Please select an operation.")
        
        plotInfo['label'] = operation + '@t=' + str(threshold)
        plotInfo['y_label'] = 'cases'
        
        # operation "true positive"
        if operation == OperationSelectorWidget.operations[0]:  
                       
            try:
                plotData = Rosbag_Analysis.getTP(self.bagFiles[0], self.bagFiles[1], threshold)
            except ValueError:
                raise Exception("Sorry, unexpected error occurred.")
             
        # operation "mismatch"
        elif operation == OperationSelectorWidget.operations[1]:
                         
            try:
                plotData = Rosbag_Analysis.getmm(self.bagFiles[0], self.bagFiles[1], threshold)
            except ValueError:
                raise Exception("Sorry, unexpected error occurred.")
             
        # operation "false positive"
        elif operation == OperationSelectorWidget.operations[2]:
                         
            try:
                plotData = Rosbag_Analysis.getFP(self.bagFiles[0], self.bagFiles[1], threshold)
            except ValueError:
                raise Exception("Sorry, unexpected error occurred.")
             
        # operation "false negative"
        elif operation == OperationSelectorWidget.operations[3]:
                         
            try:
                plotData = Rosbag_Analysis.getFN(self.bagFiles[0], self.bagFiles[1], threshold)
            except ValueError:
                raise Exception("Sorry, unexpected error occurred.")
             
        # operation "precision"
        elif operation == OperationSelectorWidget.operations[4]:
                         
            try:
                plotData = Rosbag_Analysis.getPrecision(self.bagFiles[0], self.bagFiles[1], threshold)
            except ValueError:
                raise Exception("Sorry, unexpected error occurred.")
             
        # operation "recall"
        elif operation == OperationSelectorWidget.operations[5]:
                         
            try:
                plotData = Rosbag_Analysis.getRecall(self.bagFiles[0], self.bagFiles[1], threshold)
            except ValueError:
                raise Exception("Sorry, unexpected error occurred.") 
        
        return plotData, plotInfo
         
        
         
        