'''
    this widget inherits from QWidget and 
    contains widgets to select the data required 
    for comparing and plotting data of all imported 
    bag files
'''

from python_qt_binding import QtCore, QtGui
from python_qt_binding.QtWidgets import (QWidget)


class CompareDataTab(QWidget):
    def __init__(self):
        super(CompareDataTab, self).__init__()
        