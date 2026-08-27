import numpy as np
from PySide6 import QtGui
from PySide6 import *
from PySide6.QtWidgets import *
import sys
import os
import sys
from .popout_graph_for_fr import Ui_MainWindow
import matplotlib
import matplotlib.pyplot as plt
import pandas
plt.rcParams.update({
    'font.size': 14,
    # white figure background so the canvas reads as a distinct widget instead
    # of blending into the grey window (a border is added on the canvas below)
    'figure.facecolor': '#ffffff',
    'axes.facecolor': '#ffffff',
    'axes.edgecolor': '#bbbbbb',
    'axes.grid': True,
    'grid.color': '#b0b0b0',
    'grid.linestyle': '--',
    'grid.linewidth': 0.6,
    'grid.alpha': 0.7,
})

import matplotlib as mpl

mpl.use('QtAgg')


from matplotlib.figure import Figure
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg, NavigationToolbar2QT




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
        self.setWindowTitle("f_R plot")
        
        
        self.refresh_button.clicked.connect(self.refresh_graph)
        
        
        
        #init mlp canvas 
        self.canvas = MatplotlibCanvas(self)
        # wrap the canvas in a framed container so the white plot area reads as a
        # distinct widget instead of blending into the grey window background
        self.canvas_frame = QFrame(self.centralwidget)
        self.canvas_frame.setObjectName("canvas_frame")
        self.canvas_frame.setStyleSheet(
            "QFrame#canvas_frame { background: #ffffff; border: 2px solid #6f6f6f;"
            " border-radius: 4px; }"
        )
        _canvas_frame_layout = QVBoxLayout(self.canvas_frame)
        _canvas_frame_layout.setContentsMargins(6, 6, 6, 6)
        _canvas_frame_layout.addWidget(self.canvas)
        self.mlp_layout.addWidget(self.canvas_frame)
        self.mlp_toolbar =  NavigationToolbar2QT(self.canvas, self.centralwidget)
        self.horizontalLayout.addWidget(self.mlp_toolbar)
        
        #project root dir (GUI_SRC_CODE/src) -- three levels up from
        #popout_graph/graph_fr.py, since this file moved one directory
        #deeper when popout_graph/ was split out of constant_shear_rate/
        self.project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        self.dir_name = os.path.join(self.project_root, "files", "results_fr.csv")
        #open results_fr.csv files
        
        self.header_text = "angular_velocity  [rad/s];mean_torque [Nm];std_mean_torque [Nm];std_torque [Nm];mean_phase [rad];std_mean_phase [rad];std_phase [rad]"
        
        self.data = pandas.read_csv(self.dir_name , sep=";", header=0).to_numpy()
        
        
        self.angular_velocity = self.data[:, 0]   # rad/s
        self.mean_torque = self.data[:, 1]          #Nm
        
        
        index = np.argsort(self.angular_velocity)
        self.angular_velocity = self.angular_velocity[index]
        self.mean_torque = self.mean_torque[index]
        
        self.canvas.axes.cla()  # clear canvas
        self.canvas.axes.set_title(r"$f_r$ Plot Diagram")
        self.canvas.axes.set_ylabel(r"Torque $M$ / Nm")
        self.canvas.axes.set_xlabel(r"Rotational velocity $\omega$ / $s^{-1}$")
        self.canvas.axes.plot(self.angular_velocity, self.mean_torque, color='r', marker="o")
        self.canvas.axes.minorticks_on()
        self.canvas.axes.grid(True, which='major', linestyle='--', linewidth=0.6,
                              color='#b0b0b0', alpha=0.8)
        self.canvas.axes.grid(True, which='minor', linestyle=':', linewidth=0.4,
                              color='#cccccc', alpha=0.6)
        self.canvas.draw()
        
    def refresh_graph(self):
        

        self.header_text = "angular_velocity  [rad/s];mean_torque [Nm];std_mean_torque [Nm];std_torque [Nm];mean_phase [rad];std_mean_phase [rad];std_phase [rad]"
        
        self.data = pandas.read_csv(self.dir_name , sep=";", header=0).to_numpy()
        
        
        self.angular_velocity = self.data[:, 0]   # rad/s
        self.mean_torque = self.data[:, 1]          #Nm
        
        index = np.argsort(self.angular_velocity)
        self.angular_velocity = self.angular_velocity[index]
        self.mean_torque = self.mean_torque[index]
        
        
        self.canvas.axes.cla()  # clear canvas
        self.canvas.axes.set_title(r"$f_r$ Plot Diagram")
        self.canvas.axes.set_ylabel(r"Torque $M$ / Nm")
        self.canvas.axes.set_xlabel(r"Rotational velocity $\omega$ / $s^{-1}$")
        self.canvas.axes.plot(self.angular_velocity, self.mean_torque, color='r', marker="o")
        self.canvas.axes.minorticks_on()
        self.canvas.axes.grid(True, which='major', linestyle='--', linewidth=0.6,
                              color='#b0b0b0', alpha=0.8)
        self.canvas.axes.grid(True, which='minor', linestyle=':', linewidth=0.4,
                              color='#cccccc', alpha=0.6)
        self.canvas.draw()
        
        


def main_graph_popout():
    app_graph_popout = QApplication(sys.argv)
    
    project_root = os.path.dirname(os.path.abspath(__file__))
    # construct icon path
    icon_path = os.path.join(project_root, "pics", "fzj.ico")
    
    app_graph_popout.setWindowIcon(QtGui.QIcon(icon_path))
    
    main_window = PlotWindow()
    main_window.show()
    sys.exit(app_graph_popout.exec())
        
if __name__ == '__main__':
    main_graph_popout()