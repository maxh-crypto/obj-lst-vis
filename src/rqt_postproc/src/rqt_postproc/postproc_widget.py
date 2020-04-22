from multiprocessing import sys

import sys

from python_qt_binding import QtCore, QtGui
from python_qt_binding.QtWidgets import (QWidget, QLabel, QPushButton, QVBoxLayout)
from bagselector_widget import BagSelectorWidget

class PostProcMainWidget(QWidget):
    '''
       this is the main GUI-Widget for the postprocessing Module
       it inherits from QWidget (Qt) 
       contains three other widgets: BagSelectorWidget, InfoSelectorWidget, PlotWidget
    '''
    def __init__(self):
        super(PostProcMainWidget, self).__init__()

        self.bagSelector = BagSelectorWidget()
        #self.infoSelector = InfoSelctorWidget()
        self.plot = PlotWidget()

        self.layout = QVBoxLayout()
        self.layout.addWidget(self.bagSelector)
        #self.layout.addWidget(self.infoSelector)
        self.layout.addWidget(self.plot)
        self.setLayout(self.layout)

        
        


