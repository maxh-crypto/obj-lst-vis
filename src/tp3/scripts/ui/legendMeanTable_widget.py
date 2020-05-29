'''
    this is a widget which replaces the figure legend 
    and shows the mean value and standard deviation 
    of each data graph in the figure 
'''

from python_qt_binding import QtCore, QtGui
from python_qt_binding.QtWidgets import QWidget, QTableWidget, QTableWidgetItem

class LegendInfoTable(QTableWidget):
    
    def __init__(self, parent=None):
        super(LegendInfoTable, self).__init__(1, 3)
        self.parent = parent
        
        # init the components:
        self.setHorizontalHeaderLabels(['Graph', 'Mean', 'Standard Deviation'])
        
        
    def addLine(self, label, mean, deviation):
        rowIdx = self.rowCount()
        self.insertRow(rowIdx)
        self.setItem(rowIdx, 0, QTableWidgetItem(label))
        self.setItem(rowIdx, 1, QTableWidgetItem(str(mean)))
        self.setItem(rowIdx, 2, QTableWidgetItem(str(deviation)))        
    
    
    def delLine(self, lineNr):
        self.removeRow(lineNr)

