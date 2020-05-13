'''
    this small widget is shown in plot dialog
    here the user can select which of the values 
    he wants to plot
'''
from python_qt_binding import QtCore, QtGui
from python_qt_binding.QtWidgets import *
from object_list_msg import obj_list_msg

# TODO: Maybe import ObjectList.msg directly for the tree structure?

class ValueSelectorWidget(QGroupBox):
    
    def __init__(self, parent=None):
        super(ValueSelectorWidget, self).__init__()
        self.parent = parent
        
        self.layout = QVBoxLayout()
        self.setTitle('3.Select Value')
        self.valueTreeWidget = QTreeWidget()
#         self.valueTreeWidget.itemSelectionChanged.connect(self.selectionChanged)
        self.buildTree()
        self.layout.addWidget(self.valueTreeWidget)
        self.setLayout(self.layout)
        
    def buildTree(self):
        '''
            fills the treeWidget
            according to the structure of ObjectList.msg
        '''
        for key in obj_list_msg:
            category = QTreeWidgetItem(self.valueTreeWidget)
            category.setText(0, key)
            for att in obj_list_msg[key]:
                attribute = QTreeWidgetItem(category)
                attribute.setText(0, att)
#                 attBtn = QRadioButton(att)
#                 category.addChild(attBtn)
            
    def getAttribute(self):
        currentItem = self.valueTreeWidget.currentItem()
        if currentItem.childCount() == 0: # selctedItem is a attribute
            return currentItem.text(0)
        else: # selected Item is not an attribute
            return ""
    
    def getCategory(self):
        currentItem = self.valueTreeWidget.currentItem()
        if currentItem.childCount() == 0: # selctedItem is a attribute
            return currentItem.parent().text(0)
        else: # selected Item is not an attribute
            return ""
        
    def getCatAndAtt(self):
        currentItem = self.valueTreeWidget.currentItem()
        if currentItem.childCount() == 0: # selctedItem is a attribute
            selectedAttribute = currentItem.text(0)
            selectedCategory = currentItem.parent().text(0)
        else: # selected Item is not an attribute
            selectedAttribute = ""
            selectedCategory = ""
        return { 'category' : selectedCategory, 
                'attribute' : selectedAttribute }
        
                

        
        
