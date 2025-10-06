import numpy as np
from PyQt5 import QtCore, QtGui
from PyQt5 import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QRegularExpressionValidator, QDoubleValidator
import sys
import os
import multiprocessing
import sys
from popout_graph import Ui_MainWindow
import matplotlib
import matplotlib.pyplot as plt
import pandas
plt.rcParams.update({'font.size': 14})

import matplotlib as mpl

mpl.use('Qt5Agg')


from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT




#MATPLOTLIB CANVAS
class MatplotlibCanvas(FigureCanvasQTAgg):
    def __init__(self, parent=None, width=14, height=14, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        super().__init__(fig)



class PlotWindow(QMainWindow, Ui_MainWindow):
    def __init__(self, parent=None):
        super().__init__()
        self.setupUi(self)
        
        #init mlp canvas 
        self.canvas = MatplotlibCanvas(self)
        self.mlp_layout.addWidget(self.canvas)
        self.mlp_toolbar =  NavigationToolbar2QT(self.canvas, self.centralwidget)
        self.horizontalLayout.addWidget(self.mlp_toolbar)
        
        #project root dir
        self.project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.dir_name = os.path.join(self.project_root, "files", "results_fr.csv")
        #open results_fr.csv files
        
        self.header_text = "angular_velocity  [rad/s];mean_torque [Nm];std_mean_torque [Nm];std_torque [Nm];mean_phase [rad];std_mean_phase [rad];std_phase [rad]"
        
        self.data = pandas.read_csv(self.dir_name , sep=";", header=0).to_numpy()

        
        self.canvas.axes.cla()  # clear canvas
        self.canvas.axes.set_title(r"$f_r$ Plot Diagram")
        self.canvas.axes.set_ylabel(r"Torque $M$")
        self.canvas.axes.set_xlabel(r"Time / $s$")
        
        
        


def main_graph_popout():
    app_graph_popout = QApplication(sys.argv)
    
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    # construct icon path
    icon_path = os.path.join(project_root, "pics", "fzj.ico")
    
    app_graph_popout.setWindowIcon(QtGui.QIcon(icon_path))
    
    main_window = PlotWindow()
    main_window.show()
    sys.exit(app_graph_popout.exec_())
        
if __name__ == '__main__':
    main_graph_popout()