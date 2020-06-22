'''
    this widget inherits from QGroupBox and 
    contains QRadioButtons to select the source (GT or camera data)
    for a new raw data graph
'''

from python_qt_binding import QtCore, QtGui
from python_qt_binding.QtWidgets import (QGroupBox, QVBoxLayout,
                                         QRadioButton, QDoubleSpinBox,
                                         QLabel)

class RawDataSelector(QGroupBox):
    
    dataSourceChanged = QtCore.Signal(int)
    
    def __init__(self, parent=None):
        super(RawDataSelector, self).__init__()
        self.parent = parent
        self.setTitle('Select bag file')
        
        # Init the components
        self.bag1RadioBtn = QRadioButton('ground truth')
        self.bag2RadioBtn = QRadioButton('camera')
        
        # connect the signals to the slots
        self.bag1RadioBtn.clicked.connect(self.btn1Clicked)
        self.bag2RadioBtn.clicked.connect(self.btn2Clicked)
        
        # layout
        layout = QVBoxLayout()
        layout.addWidget(self.bag1RadioBtn)
        layout.addWidget(self.bag2RadioBtn)
        self.setLayout(layout)
        
    
    def btn1Clicked(self):
        '''
            "ground truth" is selected
        '''
        self.dataSourceChanged.emit(0)   
        
        
    def btn2Clicked(self):
        '''
            "camera" is selected
        '''
        self.dataSourceChanged.emit(1)   
            
    
    
    
    