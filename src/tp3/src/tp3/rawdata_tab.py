'''
    this widget inherits from QWidget and 
    contains widgets to select the data required 
    for plotting raw data
'''

from python_qt_binding import QtCore, QtGui
from python_qt_binding.QtWidgets import QWidget, QHBoxLayout

class RawDataTab(QWidget):
    
    def __init__(self):
        super(RawDataTab, self).__init__()
        self.layout = QHBoxLayout()
        
    def initBagSelector(self):
        pass
        
    def initValueSelector(self):
        pass
            
    def initAxisSelector(self):
        pass
        
        