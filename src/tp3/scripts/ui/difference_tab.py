'''
    this widget inherits from QWidget and 
    contains widgets to select the data required 
    for plotting raw data
'''

from python_qt_binding import QtCore, QtGui
from python_qt_binding.QtWidgets import (QWidget, QGridLayout, 
                                         QVBoxLayout, QPushButton,
                                         QRadioButton, QFrame, QGroupBox,
                                         QDoubleSpinBox)

from valueSelector_widget import ValueSelectorWidget
from id_selector_widget import IDSelectorWidget
from rawDataSelector_box import RawDataSelector
from ThresholdSetter_box import ThresholdSetter
from Rosbag_Analysis import Rosbag_Analysis
import object_list_msg 
import message_module

class DiffTab(QWidget):
    
    def __init__(self, bagFiles, parent=None):
        super(DiffTab, self).__init__()
        self.parent = parent
        self.layout = QGridLayout()
    
        self.bagFiles = bagFiles
        self.selectedValue = ('', '') # contains value as a tupel like ('<message>', '<value>')      
        
        # init the widgets   
        self.thresholdSetter = ThresholdSetter(self)
        self.valueWidget = ValueSelectorWidget(self, False)        
        self.idSelector = IDSelectorWidget()
        self.idSelector.setTitle("3.Select GT-ObjectID")
        
        # connect the signals to the slots)
        self.valueWidget.valueTreeWidget.itemClicked.connect(self.valueSelected)
        
        #layout        
        self.layout.addWidget(self.thresholdSetter, 0, 0)
        self.layout.addWidget(self.valueWidget, 0, 1)
        self.layout.addWidget(self.idSelector, 0, 2)
        self.setLayout(self.layout)
        
        
    def valueSelected(self, item):
        '''
            is called when the tree in the value selector widget is clicked
            the id selector should be disabled if the item "object_count" is clicked
        '''
        if item.text(0) == "object_count":
            self.idSelector.setEnabled(False)
            self.thresholdSetter.setEnabled(False)
            
        else:
            self.idSelector.setEnabled(True)
            self.thresholdSetter.setEnabled(True)
            # init the idSelector
            if self.bagFiles[0] == '' or self.bagFiles[1] == '':
                message_module.showMessage("Bag file missing! Please import bag file in the main interface.")
            
            else:
                try: 
                    self.idSelector.refreshList(self.bagFiles[0])
                except:
                    message_module.showMessage("Object_IDs could not be parsed. Maybe there is a problem with the selected bag file.")
            
                
    def getPlotData(self):
        
        plotInfo = {
            'label' : '',
            'y_label' : '',
            'bag' : 1
            }
        
        for bag in self.bagFiles:
            if bag == "":
                raise Exception("Bag file missing! Please import bag file in the main interface.")
            
        threshold = self.thresholdSetter.getThreshold()
        
        selectedValue = self.valueWidget.getCatAndAtt()
        category = selectedValue['category']
        attribute = selectedValue['attribute']  
        # check whether attribute is empty
        # show error message when it is the case
        # and return the function
        if attribute == "":
            raise Exception("Please select a plottable attribute.") 
        
        if attribute == "object_count":
            
            # get object_counts per frame and build the difference
            # TODO: Methode von Christoph schreiben lassen mit zeitl. mapping
            try:    
                time_list_gt, obj_count_list_gt = Rosbag_Analysis.getObjectCountPerFrame(self.bagFiles[0])
                time_list_cam, obj_count_list_cam = Rosbag_Analysis.getObjectCountPerFrame(self.bagFiles[1])
            except:
                raise Exception("Sorry, unexpected error occurred.")
            
            plotData = (time_list_gt, (obj_count_list_gt - obj_count_list_cam))
            
            plotInfo['label'] = 'difference' + '.'
            plotInfo['label'] += 'object_count'
            
        # selected attribute is from object_list_msg    
        else: 
            try:
                obj_id = self.idSelector.getID()
            except Exception:
                    raise Exception("ObjectID is not in the list! Insert valid ID.")
            
            try:
                plotData = Rosbag_Analysis.getAdvancedData(self.bagFiles[0], self.bagFiles[1], obj_id, category, attribute, 'difference', threshold)
            except ValueError:
                raise Exception("Sorry, unexpected error occurred.")
        
            plotInfo['label'] = 'difference' + '.'   
            plotInfo['label'] += category + '.'
            plotInfo['label'] += attribute
            
            if object_list_msg.units.has_key(attribute):
                plotInfo['label'] += object_list_msg.units[attribute]                    
                plotInfo['y_label'] = object_list_msg.values_units[attribute]
            
        return plotData, plotInfo
