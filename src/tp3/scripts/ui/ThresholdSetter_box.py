'''
    in this GroupBox the user can set the IoU Treshold
'''

from python_qt_binding import QtCore, QtGui
from python_qt_binding.QtWidgets import QGroupBox, QDoubleSpinBox, QVBoxLayout

class ThresholdSetter(QGroupBox):
    
    def __init__(self, parent=None):
        super(ThresholdSetter, self).__init__()
        self.parent = parent
        self.setTitle('1.Select IoU Threshold ')
        self.layout = QVBoxLayout()
        self.spinBox = QDoubleSpinBox()
        self.spinBox.setDecimals(2)
        self.spinBox.setMaximum(1.0)
        self.spinBox.setSingleStep(0.1)
        self.spinBox.setValue(0.5)
        self.layout.addWidget(self.spinBox)
        self.setLayout(self.layout)
        
    def getThreshold(self):
        return self.spinBox.value()