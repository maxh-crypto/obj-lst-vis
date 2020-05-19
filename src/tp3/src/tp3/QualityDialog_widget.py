'''
    This dialog shows up when the user presses the 
    'Compute Data Quality' Button in the main window.
    It contains a spin box to adjust the threshold value
    and label which present the values FPPI, MOTA, MOTP
'''

from pyhton_qt_binding import QtCore, QtGui
from pyhton_qt_binding.QtWidget import (QDialog, QVBoxLayout, 
                                        QLabel, QDoubleSpinBox,
                                        QPushButton)

from Rosbag_Analysis import Rosbag_Analysis
import message_module

class QualityDialog(QDialog):
    
    def __init__(self, bagFiles, parent=None):
        super(QualityDialog, self).__init__()
        self.parent = parent
        self.bagFiles = bagFiles
        self.setWindowTitle("Data Quality")
        
        ## Widgets:
        self.spinBox = QDoubleSpinBox()
        self.spinBox.setDecimals(2)
        self.spinBox.setMaximum(1.0)
        self.spinBox.setSingleStep(0.1)
        self.spinBox.setValue(0.5)
        
        self.fppiLbl = QLabel("FPPI = ")
        
        self.motaLbl = QLabel("MOTA = ")
        
        self.motpLbl = QLabel("MOTP = ")
        
        self.recalcBtn = QPushButton("Recalculate Quality")
        self.recalcBtn.clicked.connect(self.calculate)
        
        ## Layout:
        layout = QVBoxLayout()
        layout.addWidget(self.spinBox)
        layout.addWidget(self.fppiLbl)
        layout.addWidget(self.motaLbl)
        layout.addWidget(self.motpLbl)
        layout.addWidget(self.recalcBtn)
        
        self.setLayout(layout)
        
        self.resize(300, 400)
        
        # calculate the data quality:
        self.calculate()
        
    
    def calculate(self):
        
        if self.bagFiles[0] == "" or self.bagFiles[1] == "":
            message_module.showMessage("Bag file missing! Please import bag file in the main interface.")  
            return
        
        threshold = self.spinBox.value()
        
        fppi = Rosbag_Analysis.getFPPI()
        self.fppiLbl.setText("FPPI = " + str(fppi))
        
        mota = Rosbag_Analysis.getMOTA()
        self.motaLbl.setText("MOTA = " + str(mota))
        
        motp = Rosbag_Analysis.getMOTP()
        self.motpLbl.setText("MOTP = " + str(motp))
        
        
        
        