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

class RawDataTab(QWidget):
    
    def __init__(self, parent=None):
        super(RawDataTab, self).__init__()
        self.parent = parent
        self.layout = QGridLayout()
    
        self.selectedBag = 2
        self.selectedValue = ('', '') # contains value as a tupel like ('<message>', '<value>')      
        
        # init the widgets
        self.bagSelector = self.initBagSelector()
        self.layout.addWidget(self.bagSelector, 1, 1)
        self.valueWidget = ValueSelectorWidget()
        self.layout.addWidget(self.valueWidget, 1, 2)
        self.idSelector = IDSelectorWidget()
        self.layout.addWidget(self.idSelector, 1, 3)
        
        self.setLayout(self.layout)
        
    def initBagSelector(self):
        '''
            determines which of the two bag file 
            should be shown in the plot
        '''
        bagSelector = QGroupBox('Select Bag')
        bagSelectorLayout = QVBoxLayout()
        bag1RadioBtn = QRadioButton('BagFile1')
        bag1RadioBtn.clicked.connect(self.btn1Clicked)
        bag2RadioBtn = QRadioButton('BagFile2')
        bag2RadioBtn.clicked.connect(self.btn2Clicked)
        bagSelectorLayout.addWidget(bag1RadioBtn)
        bagSelectorLayout.addWidget(bag2RadioBtn)
        bagSelector.setLayout(bagSelectorLayout)
        
        return bagSelector
    
    def btn1Clicked(self):
        self.selectedBag = 0
        
    def btn2Clicked(self):
        self.selectedBag = 1
              
        
        
        
        
        
        
