'''
    dialog widget for deletion of specific axes in the figure
    shows a list of all current shown axes by their labels
    user can select a line in this list and this axes is deleted
    after pressing the delete button
'''
import message_module

from python_qt_binding import QtCore, QtGui
from python_qt_binding.QtWidgets import (QDialog, QListWidget, 
                                        QVBoxLayout, QPushButton)

class DelPlotDialog(QDialog):

    deletePressed = QtCore.Signal(int)
    
    def __init__(self, linesList, parent=None):
        super(DelPlotDialog, self).__init__()
        self.linesList = linesList # list of the axes
        self.parent = parent
        
        # init the components:
        self.setWindowTitle("Delete Graph")
        self.resize(300, 400)
        self.linesListWidget = QListWidget(self)
        self.refreshList(self.linesList)
        self.__btn = QPushButton("Delete")
        
        # connect signals
        self.__btn.clicked.connect(self.btnPressed)
        
        # layout:
        layout = QVBoxLayout()
        layout.addWidget(self.linesListWidget)
        layout.addWidget(self.__btn)
        self.setLayout(layout)
        
    
    def refreshList(self, newLinesList):
        self.linesListWidget.clear()
        for line in newLinesList:
            label = line.get_label()
            self.linesListWidget.addItem(label)
            
    def btnPressed(self):
        
        if self.linesListWidget.count() == 0:
            # empty list
            return
        
        try:
            lineNr = self.linesListWidget.currentRow()
        except:
            message_module.showMessage("Please select a graph to delete.")
            
        self.linesListWidget.takeItem(lineNr) 
        self.deletePressed.emit(lineNr)
        
        