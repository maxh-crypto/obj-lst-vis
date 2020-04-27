'''
    this small widget is shown in plot dialog
    here the user can select which of the values 
    he wants to plot
'''

from python_qt_binding import QtCore, QtGui
from python_qt_binding.QtWidgets import QGroupBox, QVBoxLayout, QLabel, QFrame

# TODO: Maybe import ObjectList.msg directly for the tree structure?

class ValueSelectorWidget(QGroupBox):
    
    def __init__(self):
        super(ValueSelectorWidget, self).__init__()
        self.layout = QVBoxLayout()
#         self.setFrameStyle(QFrame.Panel)
        self.setTitle('3.Select Value')
        # TODO: add tree view with the content of ObjectList.msg 
        # + more advanced analysis like object count an so on 
        # differences can occur between time plot and objID plot
        self.setLayout(self.layout)
        