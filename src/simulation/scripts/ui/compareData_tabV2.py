'''
    this widget inherits from QWidget and 
    contains widgets to select the data required 
    for comparing and plotting data of all imported 
    bag files
'''

import threading
from python_qt_binding import QtCore, QtGui
from python_qt_binding.QtWidgets import (QWidget, QHBoxLayout, QMessageBox)
from operationSelector_box import OperationSelectorWidget
from Rosbag_Analysis import Rosbag_Analysis
from ThresholdSetter_box import ThresholdSetter
from computingDialog_widget import ComputationDialog

import object_list_msg

class CompareDataTab(QWidget):
    
    def __init__(self, bagFiles, parent=None):
        super(CompareDataTab, self).__init__()
        self.parent = parent
        
        # attributes
        self.bagFiles = bagFiles
        self.plotData = ([], [], 0.0, 0.0)
        self.plotInfo = {
            'label' : '',
            'y_label' : '',
            }
        
        # widgets
        self.operationSelector = OperationSelectorWidget(self)
        self.thresholdSetter = ThresholdSetter(self)
        self.waitMessageBox = QMessageBox(self)
        self.waitMessageBox.setIcon(QMessageBox.Information)
        self.waitMessageBox.setWindowTitle('Computation')
        
        # layout:
        layout = QHBoxLayout()
        layout.addWidget(self.operationSelector)
        layout.addWidget(self.thresholdSetter)
        
        self.setLayout(layout)
            
            
    def getPlotData(self):
        '''
            returns plotData and plotInfo 
        '''
        
        if self.bagFiles[0] == '' or self.bagFiles[1] == '':
            raise Exception("Bag file missing. Please import two bag files in the main interface.")
        
        threshold = self.thresholdSetter.getThreshold()
        
        # get operation
        operation = self.operationSelector.getOperation()
        
        if operation == '':
            raise Exception("Please select an operation.")
        
        self.plotInfo['label'] = operation + '@t=' + str(threshold)
        self.plotInfo['y_label'] = 'cases'
        
        # start new thread
        thread = EvaluationThread(self, operation, self.bagFiles, threshold)
#         thread.finished.connect(self.waitMessageBox.close)
        thread.start()
#         self.waitMessageBox.setText('Computing ' + operation + ' values... \nThis could take a few seconds')
#         
#         if thread.isRunning():
#             self.waitMessageBox.exec_()
        
        return self.plotData, self.plotInfo
        
        
class EvaluationThread(QtCore.QThread):
     
    def __init__(self, mainWidget, operation, bagFiles, threshold):
        QtCore.QThread.__init__(self)
        self.operation = operation
        self.bagFiles = bagFiles
        self.threshold = threshold
        self.mainWidget = mainWidget
         
    def run(self):
         
       # operation "true positive"
        if self.operation == OperationSelectorWidget.operations[0]:  
                       
            try:
                plotData = Rosbag_Analysis.getTP(self.bagFiles[0], self.bagFiles[1], self.threshold)
            except ValueError:
                raise Exception("Sorry, unexpected error occurred.")
             
        # operation "mismatch"
        elif self.operation == OperationSelectorWidget.operations[1]:
                         
            try:
                plotData = Rosbag_Analysis.getmm(self.bagFiles[0], self.bagFiles[1], self.threshold)
            except ValueError:
                raise Exception("Sorry, unexpected error occurred.")
             
        # operation "false positive"
        elif self.operation == OperationSelectorWidget.operations[2]:
                         
            try:
                plotData = Rosbag_Analysis.getFP(self.bagFiles[0], self.bagFiles[1], self.threshold)
            except ValueError:
                raise Exception("Sorry, unexpected error occurred.")
             
        # operation "false negative"
        elif self.operation == OperationSelectorWidget.operations[3]:
                         
            try:
                plotData = Rosbag_Analysis.getFN(self.bagFiles[0], self.bagFiles[1], self.threshold)
            except ValueError:
                raise Exception("Sorry, unexpected error occurred.")
             
        # operation "precision"
        elif self.operation == OperationSelectorWidget.operations[4]:
                         
            try:
                plotData = Rosbag_Analysis.getPrecision(self.bagFiles[0], self.bagFiles[1], self.threshold)
            except ValueError:
                raise Exception("Sorry, unexpected error occurred.")
             
        # operation "recall"
        elif self.operation == OperationSelectorWidget.operations[5]:
                         
            try:
                plotData = Rosbag_Analysis.getRecall(self.bagFiles[0], self.bagFiles[1], self.threshold)
            except ValueError:
                raise Exception("Sorry, unexpected error occurred.") 
        
        self.mainWidget.plotData = plotData    
        self.terminate()
         
        