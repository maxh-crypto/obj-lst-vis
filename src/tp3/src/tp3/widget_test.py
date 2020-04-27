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
    w.setWindowTitle("Object List Postprocessing")
    w.show()
    sys.exit(app.exec_())
    
if __name__ == '__main__':
    window()