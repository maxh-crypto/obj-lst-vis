'''
    dialog widget for selecting the information 
    that should be presented in a new plot
'''

from python_qt_binding import QtCore, QtGui
from python_qt_binding.QtWidgets import QDialog, QTabWidget, QVBoxLayout
from rawdata_tab import RawDataTab

class PlotDialogWidget(QDialog):
    
    def __init__(self):
        super(PlotDialogWidget, self).__init__()
        self.setWindowTitle("Add new Plot")
        # self.setWindowModality(Qt.ApplicationModal)
        self.layout = QVBoxLayout()        
        self.resize(600, 400)
        
        # TabWidget
        self.tabWidget = QTabWidget()
        self.rawDataTab = RawDataTab()
        self.tabWidget.addTab(self.rawDataTab, "Raw Data")
        self.layout.addWidget(self.tabWidget)
        
        self.setLayout(self.layout)
        

