'''
    in this GroupBox the user can select 
    which evaluation function he wants to perform 
    on the imported Rosbag files
'''

from python_qt_binding import QtCore, QtGui
from python_qt_binding.QtWidgets import QGroupBox, QVBoxLayout, QRadioButton

class OperationSelectorWidget(QGroupBox):
    
    selectionChanged = QtCore.Signal(str)
    
    operations = [
                  'true positive', 
                  'mismatch',
                  'false positive', 
                  'false negative', 
                  'precision',
                  'recall'
                  ]
    
    
    def __init__(self, parent=None):
        super(OperationSelectorWidget, self).__init__()
        self.parent = parent
        self.setTitle('Select Value')
        
        self.layout = QVBoxLayout()
        self.initRadioButtons()
        self._currentSelected = ''
        
        self.setLayout(self.layout)
        
        
    def initRadioButtons(self):
        for operation in self.operations:
            btn = QRadioButton(operation)
            btn.clicked.connect(lambda state, x=operation: self.switchOperation(x))  
            self.layout.addWidget(btn)
            
            
    def switchOperation(self, operation):
        self._currentSelected = operation
        self.selectionChanged.emit(operation)
        
        
    def getOperation(self):
        return self._currentSelected
        
            
        