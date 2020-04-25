from multiprocessing import sys

import sys

from python_qt_binding import QtCore, QtGui
from python_qt_binding.QtWidgets import (QWidget, QLabel, QPushButton, QGridLayout)
from bagselector_widget import BagSelectorWidget
from plot_widget import PlotWidget

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

        self.layout = QGridLayout()
        self.layout.addWidget(self.bagSelector, 1, 1)
        #self.layout.addWidget(self.infoSelector)
        self.layout.addWidget(self.plot, 2, 1)
        self.setLayout(self.layout)

        
        


