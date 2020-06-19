'''
    this widget inherits from QWidget and 
    contains widgets to select the data required 
    for plotting raw data
    it is a tab embedded in the plot dialog widget
'''
from python_qt_binding import QtCore, QtGui
from python_qt_binding.QtWidgets import (QWidget, QGridLayout, 
                                         QVBoxLayout, QPushButton,
                                         QRadioButton, QFrame, QGroupBox,
                                         QDoubleSpinBox)

from valueSelector_widget import ValueSelectorWidget
from id_selector_widget import IDSelectorWidget
from rawDataSelector_box import RawDataSelector
from Rosbag_Analysis import Rosbag_Analysis
import object_list_msg 
import message_module

class RawDataTab(QWidget):
    
    def __init__(self, bagFiles, parent=None):
        super(RawDataTab, self).__init__()
        self.parent = parent
        self.layout = QGridLayout()
    
        self.bagFiles = bagFiles
        self.selectedSource = 2 # no bag is selected
        self.selectedValue = ('', '') # contains value as a tupel like ('<message>', '<value>')      
        
        # init the widgets
        self.sourceSelector = RawDataSelector(self)
        self.layout.addWidget(self.sourceSelector, 0, 0)
        self.valueWidget = ValueSelectorWidget()
        self.layout.addWidget(self.valueWidget, 0, 1)
        self.idSelector = IDSelectorWidget()
        self.layout.addWidget(self.idSelector, 0, 2)
        
        # connect the signals to the slots
        self.sourceSelector.dataSourceChanged.connect(self.sourceChanged)
        self.valueWidget.valueTreeWidget.itemClicked.connect(self.valueSelected)
        
        self.setLayout(self.layout)
        
         
    def sourceChanged(self, source):
        '''
            if the source in rawDataSelector is changed
            this slot is called
        '''        
        self.selectedSource = source
        
        if source == 0: # ground truth is selected
            self.idSelector.setTitle("Select GT-ObjectID")
            try: 
                self.idSelector.refreshList(self.bagFiles[0])
            except:
                message_module.showMessage("Object_IDs could not be parsed. Maybe there is a problem with the selected bag file.")
        
        elif source == 1: # camera is selected
            self.idSelector.setTitle("Select Cam-ObjectID")
            try: 
                self.idSelector.refreshList(self.bagFiles[1])
            except:
                message_module.showMessage("Object_IDs could not be parsed. Maybe there is a problem with the selected bag file.")
                
                
    def valueSelected(self, item):
        '''
            is called when the tree in the value selector widget is clicked
            the id selector should be disabled if the item "object_count" is clicked
        '''
        if item.text(0) == "object_count":
            self.idSelector.setEnabled(False)
            
        else:
            self.idSelector.setEnabled(True)
    
    
    def getPlotData(self):
        '''
            provides the raw data according to the selected parameters
        '''
        # info needed for the figure legend
        plotInfo = {
            'label' : '',
            'y_label' : '',
            'bag' : 1
            }
        
        if self.selectedSource > 1:
            raise Exception("No source bag selected. Please select a bag file.")
        
        selectedValue = self.valueWidget.getCatAndAtt()
        category = selectedValue['category']
        attribute = selectedValue['attribute']  
        # check whether attribute is empty
        # show error message when it is the case
        # and return the function
        if attribute == "":
            raise Exception("Please select a plottable attribute.") 
              
        bagfile = self.bagFiles[self.selectedSource]
        if bagfile == "":
            raise Exception("no bag file loaded! Please import bag file in the main interface.")
        
        if attribute == "object_count":
            # get object count per frame
            try:    
                plotData = Rosbag_Analysis.getObjectCountPerFrame(bagfile)
            except:
                raise Exception("Sorry, unexpected error occurred.")
            
            bag_id = self.selectedSource + 1                 
            plotInfo['label'] = 'bag' + str(bag_id) + '.'
            plotInfo['label'] += 'object_count'
        
        # selected attribute is from object_list_message    
        else:
            try:
                obj_id = self.idSelector.getID()
            except Exception:
                raise Exception("ObjectID is not in the list. Insert valid ID.")       
                        
            try:    
                plotData = Rosbag_Analysis.getRawData(bagfile, obj_id, category, attribute)
            except:
                raise Exception("Sorry, unexpected error occurred.")
            
            bag_id = self.selectedSource + 1 
            plotInfo['label'] = 'bag' + str(bag_id) + '.'
            plotInfo['label'] += 'obj' + str(obj_id) + '.'
            
            if category != '':
                plotInfo['label'] += category + '.'
                
            plotInfo['label'] += attribute
            
            plotInfo['bag'] = bag_id
            
            if object_list_msg.units.has_key(attribute):
                plotInfo['label'] += object_list_msg.units[attribute]
                plotInfo['y_label'] = object_list_msg.values_units[attribute]                     

        
        return plotData, plotInfo
       
        
        
