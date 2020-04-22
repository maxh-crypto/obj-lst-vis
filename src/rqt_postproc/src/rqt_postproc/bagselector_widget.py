from python_qt_binding import QtCore, QtGui
from python_qt_binding.QtWidgets import QWidget, QLineEdit, QPushButton, QGridLayout

class BagSelectorWidget(QWidget):    
    def __init__(self):
        super(BagSelectorWidget, self).__init__()
        
        # create elements
        self.bag1Edit = QLineEdit()
        self.bag1Btn = QPushButton("bag file 1")
        self.bag2Edit = QLineEdit()
        self.bag2Btn = QPushButton("bag file 2")
        
        self.layout = QGridLayout()
        self.layout.addWidget(self.bag1Edit, 1, 1)
        self.layout.addWidget(self.bag1Btn, 1, 2)
        self.layout.addWidget(self.bag2Edit, 2, 1)
        self.layout.addWidget(self.bag2Btn, 2, 2)
        self.setLayout(self.layout)
