'''
    a module for error messages in an extra window
'''

from python_qt_binding import QtCore, QtGui
from python_qt_binding.QtWidgets import QMessageBox

def showMessage(msgText):
        msgBox = QMessageBox()
        msgBox.setText(msgText)
        msgBox.exec_()
        

class MessageBox(QMessageBox):
    
    def __init__(self, text):
        super(MessageBox, self).__init__()
        self.setText()