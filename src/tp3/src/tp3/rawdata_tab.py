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
import message_module

class RawDataTab(QWidget):
    
    def __init__(self, bagFiles, parent=None):
        super(RawDataTab, self).__init__()
        self.parent = parent
        self.layout = QGridLayout()
    
        self.bagFiles = bagFiles
        self.selectedBag = 3 # no bag is selected
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
        bagSelector = QGroupBox('1.Select Bag or diff.')
        bagSelectorLayout = QVBoxLayout()
        bag1RadioBtn = QRadioButton('ground truth')
        bag1RadioBtn.clicked.connect(self.btn1Clicked)
        bag2RadioBtn = QRadioButton('camera')
        bag2RadioBtn.clicked.connect(self.btn2Clicked)
        diffBtn = QRadioButton('diff. (both)')
        diffBtn.clicked.connect(self.diffBtnClicked)
        bagSelectorLayout.addWidget(bag1RadioBtn)
        bagSelectorLayout.addWidget(bag2RadioBtn)
        bagSelectorLayout.addWidget(diffBtn)
        bagSelector.setLayout(bagSelectorLayout)
        
        return bagSelector
    
    def btn1Clicked(self):
        self.selectedBag = 0
        self.idSelector.setTitle("3.Select GT-ObjectID")
        self.idSelector.refreshList(self.bagFiles[0])
        
    def btn2Clicked(self):
        self.selectedBag = 1
        self.idSelector.setTitle("3.Select Cam-ObjectID")
        self.idSelector.refreshList(self.bagFiles[1])
        
    def diffBtnClicked(self):
        self.selectedBag = 2
        self.idSelector.setTitle("3.Select GT-ObjectID")
        self.idSelector.refreshList(self.bagFiles[0]) # gt objects are used
        
    def getPlotData(self):
        '''
            gets the raw data according to the selected parameters
            from Rosbag_Analysis
        '''
        plotInfo = {
            'label' : '',
            'y_label' : '',
            'bag' : 1
            }
        
        selectedValue = self.valueWidget.getCatAndAtt()
        category = selectedValue['category']
        attribute = selectedValue['attribute']  
        # check whether attribute is empty
        # show error message when it is the case
        # and return the function
        if attribute == "":
            raise Exception("Please select a plottable attribute.")            
        
        try:
            obj_id = self.idSelector.getID()
        except ValueError:
            raise Exception("ObjectID is not a number! Insert valid ID.")
        
        if self.selectedBag < 2: # a single bag should be analysed
            
            
            bagfile = self.bagFiles[self.selectedBag]
            if bagfile == "":
                raise Exception("no bag file loaded! Please import bag file in the main interface.")
                        
            try:    
                plotData = Rosbag_Analysis.getRawData(bagfile, obj_id, category, attribute)
            except:
                raise Exception("Sorry, unexpected error occurred.")
            
            bag_id = self.selectedBag + 1
            plotInfo['label'] = 'bag' + str(bag_id) + '.'
            plotInfo['label'] += 'obj' + str(obj_id) + '.'
            plotInfo['label'] += category + '.'
            plotInfo['label'] += attribute
            
            plotInfo['y_label'] = object_list_msg.units[attribute]
            
            plotInfo['bag'] = bag_id
        
        elif self.selectedBag == 2: # difference is selected
            
            for bag in self.bagFiles:
                if bag == "":
                    raise Exception("Bag file missing! Please import bag file in the main interface.")
            
            try:
                plotData = Rosbag_Analysis.getAdvancedData(self.bagFiles[0], self.bagFiles[1], obj_id, category, attribute, 'difference')
            except ValueError:
                raise Exception("Sorry, unexpected error occurred.")
            
            plotInfo['label'] = operation + '.'
            plotInfo['label'] += 'obj' + str(obj_id) + '.'
            plotInfo['label'] += category + '.'
            plotInfo['label'] += attribute
            
            plotInfo['y_label'] = object_list_msg.units[attribute]
        
        else: # no bag file is selected
            raise Exception("Please select a bag file or difference.")
        
        return plotData, plotInfo
    
        
        
        
        
