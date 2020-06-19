# deprecated, currently not in use

'''
	this small widget is shown in the plot dialog
	here the user can determine the type of the xAxis value
'''

from python_qt_binding import QtCore, QtGui
from python_qt_binding.QtWidgets import (QGroupBox, QVBoxLayout, 
										QButtonGroup, QRadioButton,
										QFrame)

from xType_enum import xAxisTypes

class AxisTypeSelectorWidget(QGroupBox):
	
	def __init__(self):
		super(AxisTypeSelectorWidget, self).__init__()
		self.layout = QVBoxLayout()
# 		self.setFrameStyle(QFrame.Panel)
		self.setTitle('2. xAxis Type')
		
		for type in (xAxisTypes):
			radiobtn = QRadioButton(type.name)
			self.layout.addWidget(radiobtn)
			
		self.setLayout(self.layout)
			
			
			