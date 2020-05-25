from python_qt_binding import QtCore, QtGui
from python_qt_binding.QtWidgets import QMessageBox

def showMessage(msgText):
        msgBox = QMessageBox()
        msgBox.setText(msgText)
        msgBox.exec_()