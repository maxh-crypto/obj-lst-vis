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
        self.setHorizontalHeaderLabels(['Graph', 'Mean', 'Std Deviation'])
        self.setColumnWidth(0, 300)
        self.setColumnWidth(1, 100)
        self.setColumnWidth(2, 100)
        
        
    def addRow(self, label, c_rgba, mean, deviation):
        rowIdx = self.rowCount() - 1
        self.insertRow(rowIdx)
        # create QColor
        color = QtGui.QColor.fromRgbF(c_rgba[0], c_rgba[1], c_rgba[2], c_rgba[3])        
        # create QBrush with color
        brush = QtGui.QBrush(color)
        
        # legend item
        leg_item = QTableWidgetItem(label)
        leg_item.setBackground(brush)        
        self.setItem(rowIdx, 0, leg_item)
        
        # mean item
        mean_item = QTableWidgetItem(str(mean))
        mean_item.setBackground(brush)
        self.setItem(rowIdx, 1, mean_item)
        
        # deviation item
        dev_item = QTableWidgetItem(str(deviation))
        dev_item.setBackground(brush)
        self.setItem(rowIdx, 2, dev_item)        
    
    
    def delLine(self, lineNr):
        self.removeRow(lineNr)

