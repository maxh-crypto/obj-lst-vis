'''
    file for testing the gui without ros
'''
import sys
from python_qt_binding import QtCore, QtGui
from python_qt_binding.QtWidgets import (QWidget, QApplication)
from postproc_widget import PostProcMainWidget

def window():
    app = QApplication(sys.argv)
    w = PostProcMainWidget()
    w.bagWidget.bagSelector1.bagEdit.setText('/home/max/obj-lst-vis/src/simulation/rosbag_files/Groundtruth_2020-6-18-8-20-53.bag')
    w.bagWidget.bagSelector1.fileName = '/home/max/obj-lst-vis/src/simulation/rosbag_files/Groundtruth_2020-6-18-8-20-53.bag'
    w.bagWidget.bagSelector2.bagEdit.setText('/home/max/obj-lst-vis/src/simulation/rosbag_files/Camera_2020-6-18-8-20-53.bag')
    w.bagWidget.bagSelector2.fileName = '/home/max/obj-lst-vis/src/simulation/rosbag_files/Camera_2020-6-18-8-20-53.bag'
    w.setWindowTitle("Object List Postprocessing")
    w.show()
    sys.exit(app.exec_())
    
if __name__ == '__main__':
    window()