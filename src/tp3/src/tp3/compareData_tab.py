'''
    this widget inherits from QWidget and 
    contains widgets to select the data required 
    for comparing and plotting data of all imported 
    bag files
'''

from python_qt_binding import QtCore, QtGui
from python_qt_binding.QtWidgets import (QWidget, QHBoxLayout)
from operationSelector_box import OperationSelectorWidget
from id_selector_widget import IDSelectorWidget
from valueSelector_widget import ValueSelectorWidget
from Rosbag_Analysis import Rosbag_Analysis

import object_list_msg

class CompareDataTab(QWidget):
    
    def __init__(self, bagFiles, parent=None):
        super(CompareDataTab, self).__init__()
        self.parent = parent
        self.bagFiles = bagFiles
        
        # widgets
        self.operationSelector = OperationSelectorWidget(self)
        self.operationSelector.selectionChanged.connect(self.operationChanged)
        self.valueSelector = ValueSelectorWidget(self)
        self.valueSelector.setEnabled(False)
        self.idSelector = IDSelectorWidget()
        self.idSelector.setEnabled(False)
        self.idSelector.refreshList(bagFiles[0])
        
        # layout:
        layout = QHBoxLayout()
        layout.addWidget(self.operationSelector)
        layout.addWidget(self.valueSelector)
        layout.addWidget(self.idSelector)
        
        self.setLayout(layout)
        
    def operationChanged(self, operation):
        if operation == self.operationSelector.operations[0]:
            self.idSelector.setEnabled(True)
            self.valueSelector.setEnabled(True)
        else:
            self.idSelector.setEnabled(False)
            self.valueSelector.setEnabled(False)
            
    def getPlotData(self):
        '''
            returns plotData and plotInfo 
        '''
        plotInfo = {
            'label' : '',
            'unit' : '',
            }
        
        # get operation
        operation = self.operationSelector.getOperation()
        
        # operation "difference":
        if operation == self.operationSelector.operations[0]:
            selectedValue = self.valueSelector.getCatAndAtt()
            category = selectedValue['category']
            attribute = selectedValue['attribute']  
            # check whether category or attribute is empty
            # show error message when it is the case
            # and exit the function
            if attribute == "":
                self.showMessage("Please select a plottable attribute.")
                return 
            
            try:
                obj_id = self.idSelector.getID()
            except ValueError:
                self.showMessage("ObjectID is not a number! Insert valid ID.")
                return
            
            try:
                plotData = Rosbag_Analysis.getAdvancedData(self.bagFiles[0], self.bagFiles[1], obj_id, category, attribute, operation)
            except ValueError:
                self.showMessage("Sorry, unexpected error occurred.")
                return
            
            plotInfo['label'] = operation + '.'
            plotInfo['label'] += 'obj' + str(obj_id) + '.'
            plotInfo['label'] += category + '.'
            plotInfo['label'] += attribute
            
            plotInfo['unit'] = object_list_msg.units[attribute]
            
        else:
            self.showMessage("Operation not supported yet.")
            return
        
        return plotData, plotInfo
        
    def showMessage(self, msgText):
        msgBox = QMessageBox()
        msgBox.setText(msgText)
        msgBox.exec_()
        
        