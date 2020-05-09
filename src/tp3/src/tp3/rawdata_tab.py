'''
    this widget inherits from QWidget and 
    contains widgets to select the data required 
    for plotting raw data
'''

from python_qt_binding import QtCore, QtGui
from python_qt_binding.QtWidgets import (QWidget, QGridLayout, 
                                         QVBoxLayout, QPushButton,
                                         QRadioButton, QFrame, QGroupBox)

from valueSelector_widget import ValueSelectorWidget
from id_selector_widget import IDSelectorWidget
from Rosbag_Analysis import Rosbag_Analysis
import object_list_msg 

class RawDataTab(QWidget):
    
    def __init__(self, bagFiles, parent=None):
        super(RawDataTab, self).__init__()
        self.parent = parent
        self.layout = QGridLayout()
    
        self.bagFiles = bagFiles
        self.selectedBag = 2 # no bag is selected
        self.selectedValue = ('', '') # contains value as a tupel like ('<message>', '<value>')      
        
        # init the widgets
        self.bagSelector = self.initBagSelector()
        self.layout.addWidget(self.bagSelector, 0, 0)
        self.valueWidget = ValueSelectorWidget()
        self.layout.addWidget(self.valueWidget, 0, 1)
        self.idSelector = IDSelectorWidget()
        self.layout.addWidget(self.idSelector, 0, 2)
        
        self.setLayout(self.layout)
        
    def initBagSelector(self):
        '''
            determines which of the two bag file 
            should be shown in the plot
        '''
        bagSelector = QGroupBox('1.Select Bag')
        bagSelectorLayout = QVBoxLayout()
        bag1RadioBtn = QRadioButton('ground truth')
        bag1RadioBtn.clicked.connect(self.btn1Clicked)
        bag2RadioBtn = QRadioButton('camera')
        bag2RadioBtn.clicked.connect(self.btn2Clicked)
        bagSelectorLayout.addWidget(bag1RadioBtn)
        bagSelectorLayout.addWidget(bag2RadioBtn)
        bagSelector.setLayout(bagSelectorLayout)
        
        return bagSelector
    
    def btn1Clicked(self):
        self.selectedBag = 0
        self.idSelector.refreshList(self.bagFiles[0])
        
    def btn2Clicked(self):
        self.selectedBag = 1
        self.idSelector.refreshList(self.bagFiles[1])
        
    def getPlotData(self):
        '''
            gets the raw data according to the selected parameters
            from Rosbag_Analysis
        '''
        plotInfo = {
            'label' : '',
            'unit' : '',
            }
        
        selectedValue = self.valueWidget.getCatAndAtt()
        category = selectedValue['category']
        attribute = selectedValue['attribute']  
        # check whether category or attribute is empty
        # show error message when it is the case
        # and return the function
        if attribute == "":
            self.showMessage("Please select a plottable attribute.")
            return 
        
        if self.selectedBag > 1: # no bag file is selected
            self.showMessage("Please select a bag file.")
            return
            
        bagfile = self.bagFiles[self.selectedBag]
        if bagfile == "":
            self.showMessage("no bag file loaded! Please import bag file in the main interface.")
            return
        
        try:
            obj_id = self.idSelector.getID()
        except ValueError:
            self.showMessage("ObjectID is not a number! Insert valid ID.")
            return
        
        try:    
            plotData = Rosbag_Analysis.getRawData(bagfile, obj_id, category, attribute)
        except:
            self.showMessage("Sorry, unexpected error occurred.")
            return
        
        bag_id = self.selectedBag + 1
        plotInfo['label'] = 'bag' + str(bag_id) + '.'
        plotInfo['label'] += 'obj' + str(obj_id) + '.'
        plotInfo['label'] += category + '.'
        plotInfo['label'] += attribute
        
        plotInfo['unit'] = object_list_msg.units[attribute]
        
        return plotData, plotInfo
    
    def showMessage(self, msgText):
        msgBox = QMessageBox()
        msgBox.setText(msgText)
        msgBox.exec_()
        
        
        
        
