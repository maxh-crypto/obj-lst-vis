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
from axisTypeSelector_widget import AxisTypeSelectorWidget
from xType_enum import xAxisTypes

class RawDataTab(QWidget):
    
    def __init__(self):
        super(RawDataTab, self).__init__()
        self.layout = QGridLayout()
        
        self.selectedBag = 0
        self.selectedValue = ('', '') # contains value as a tupel like ('<message>', '<value>')
        self.xAxisType = xAxisTypes.NotSelected        
        
        # init the widgets
        self.bagSelector = self.initBagSelector()
        self.layout.addWidget(self.bagSelector, 1, 1)
        self.axisTypeSelector = AxisTypeSelectorWidget()
        self.layout.addWidget(self.axisTypeSelector, 1, 2)
        self.valueWidget = ValueSelectorWidget()
        self.layout.addWidget(self.valueWidget, 1, 3)
        
        # init the start button
        startBtn = QPushButton("Start")
        startBtn.clicked.connect(self.getPlotData)
        self.layout.addWidget(startBtn, 2, 3)
        
        self.setLayout(self.layout)
        
    def initBagSelector(self):
        '''
            determines which of the two bag file 
            should be shown in the plot
        '''
        bagSelector = QGroupBox('Select Bag')
        bagSelectorLayout = QVBoxLayout()
        bag1RadioBtn = QRadioButton('BagFile1')
        bag2RadioBtn = QRadioButton('BagFile2')
        bagSelectorLayout.addWidget(bag1RadioBtn)
        bagSelectorLayout.addWidget(bag2RadioBtn)
        bagSelector.setLayout(bagSelectorLayout)
        
        return bagSelector
              
    
    def getPlotData(self):
        # is called when start button is clicked
        # gets an Array from 
        pass
        
        