'''
    this small widget is shown in plot dialog
    here the user can select which object he 
    wants to plot
    contains a text edit widget to write the id manually 
    and a list of all possible IDs, that appear in the selected bag
'''

from python_qt_binding import QtCore, QtGui
from python_qt_binding.QtWidgets import *
from Rosbag_Analysis import Rosbag_Analysis

class IDSelectorWidget(QGroupBox):
    
    def __init__(self, parent=None):
        super(IDSelectorWidget, self).__init__()
        self.parent = parent
        self.setTitle('Select ObjectID')
        
        self.idList = [] # list of strings with all ids in the selected bag
        
        # init the items
        self.lineEdit = QLineEdit()
        self.idListWidget = QListWidget()
        self.idListWidget.currentItemChanged.connect(self.idSelected)
        
        # layout:
        layout = QVBoxLayout()
        layout.addWidget(self.lineEdit)
        layout.addWidget(self.idListWidget)
        self.setLayout(layout)
        
    def getID(self):
        id_str = self.lineEdit.text()
        id_str = id_str.strip() # remove the whitespace before and after the id
        
        if id_str not in self.idList:
            raise Exception('Item not in list')
        
        id = int(id_str)
                
        return id
    
    
    def refreshList(self, bagFileName):
        '''
            refreshes the list with the ids of the current 
        '''
        
        if bagFileName == "":
            return
        try:
            ids = Rosbag_Analysis.getObjectIDs(bagFileName)
        except:
            raise Exception("Object_IDs could not be parsed. Maybe there is a problem with the selected bag file.")
        
        self.idList = map(str, ids) # convert list to strings
        self.idListWidget.clear()
        self.idListWidget.addItems(self.idList)
        
        
    def idSelected(self, curItem, prevItem):
        '''
            is called when list item is clicked
            fills the lineEdit with the clicked id
        '''
        self.lineEdit.setText(curItem.text())
        
        
        
        
        