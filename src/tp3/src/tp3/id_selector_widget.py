'''
    this small widget is shown in plot dialog
    here the user can select which object he 
    wants to plot
    contains a text edit widget to write the id manually (with autofill)
    and a list of all possible IDs, that appear in the selected bag
'''

from python_qt_binding import QtCore, QtGui
from python_qt_binding.QtWidgets import *

class IDSelectorWidget(QGroupBox):
    
    def __init__(self, parent=None):
        super(IDSelectorWidget, self).__init__()
        self.parent = parent
        self.setTitle('3.Select ObjectID')
        layout = QVBoxLayout()
        
        self.lineEdit = QLineEdit()
        self.idList = QListWidget()
        
        layout.addWidget(self.lineEdit)
        layout.addWidget(self.idList)
        self.setLayout(layout)
        
    def getID(self):
        id = int(self.lineEdit.text())
        return id
        