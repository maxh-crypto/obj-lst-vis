import matplotlib.pyplot as plt
import numpy as np

from python_qt_binding import QtCore, QtGui
from python_qt_binding.QtWidgets import QWidget, QLineEdit, QPushButton, QVBoxLayout

from matplotlib.backends.qt_compat import QtCore, QtWidgets, is_pyqt5
if is_pyqt5():
    from matplotlib.backends.backend_qt5agg import (
        FigureCanvas
        #, NavigationToolbar2QT as NavigationToolbar
        )
else:
    from matplotlib.backends.backend_qt4agg import (
        FigureCanvas
        #, NavigationToolbar2QT as NavigationToolbar
        )
from matplotlib.figure import Figure

class PlotWidget(QWidget):
    def __init__(self):
        super(PlotWidget, self).__init__()
        self.layout = QVBoxLayout()
        canvas = FigureCanvas(Figure(figsize=(5, 3)))
        self.layout.addWidget(canvas)
        self.ax = canvas.figure.subplots()
        t = np.linspace(0, 10, 101)
        self.ax.set_ylim(-1.1, 1.1)
        self.ax.plot(t, sin(t))
        self.ax.figure.canvas.draw()
        
        