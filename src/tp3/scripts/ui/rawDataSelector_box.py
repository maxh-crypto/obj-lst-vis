'''
    this widget inherits from QGroupBox and 
    contains QRadioButtons to select the source for a
    new raw data graph
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
        self.setTitle('1.Select Bag or diff.')
        
        # Init the components
        self.bag1RadioBtn = QRadioButton('ground truth')
        self.bag2RadioBtn = QRadioButton('camera')
#         self.diffBtn = QRadioButton('diff. (gt - cam)')
#         self.thresholdLabel = QLabel("Threshold:")
#         self.thresholdLabel.setEnabled(False)
#         self.thresholdSetter = QDoubleSpinBox()
#         self.thresholdSetter.setDecimals(2)
#         self.thresholdSetter.setMaximum(1.0)
#         self.thresholdSetter.setSingleStep(0.1)
#         self.thresholdSetter.setValue(0.5)
#         self.thresholdSetter.setEnabled(False)
        
        # connect the signals to the slots
        self.bag1RadioBtn.clicked.connect(self.btn1Clicked)
        self.bag2RadioBtn.clicked.connect(self.btn2Clicked)
#         self.diffBtn.clicked.connect(self.diffBtnClicked)
        
        # layout
        layout = QVBoxLayout()
        layout.addWidget(self.bag1RadioBtn)
        layout.addWidget(self.bag2RadioBtn)
#         layout.addWidget(self.diffBtn)
#         self.thresholdLabel.setMaximumHeight(15)     
#         layout.addWidget(self.thresholdLabel)
#         layout.addWidget(self.thresholdSetter)
        self.setLayout(layout)
        
    
    def btn1Clicked(self):
        '''
            "ground truth" is selected
        '''
#         self.thresholdLabel.setEnabled(False) 
#         self.thresholdSetter.setEnabled(False)
        self.dataSourceChanged.emit(0)   
        
        
    def btn2Clicked(self):
        '''
            "camera" is selected
        '''
#         self.thresholdLabel.setEnabled(False)
#         self.thresholdSetter.setEnabled(False) 
        self.dataSourceChanged.emit(1)   
            
        
#     def diffBtnClicked(self):
#         '''
#             "difference" is selected
#         '''
#         self.thresholdLabel.setEnabled(True)
#         self.thresholdSetter.setEnabled(True) 
#         self.dataSourceChanged.emit(2)   
        
#             
#     def getThreshold(self):
#         return self.thresholdSetter.value()
    
    
    
    