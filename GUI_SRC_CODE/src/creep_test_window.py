import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5 import *
from PyQt5.QtCore import QThread, pyqtSignal, QMutex, QRegularExpression, QObject, QTimer
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QRegularExpressionValidator, QDoubleValidator
import sys
import os
from pathlib import Path


# #fix cache problem with MATHPLOTLIB
os.environ['MPLCONFIGDIR'] = str(Path.home()) +"/.matplotlib/"
import multiprocessing
import sys

import sockets_files 
from sockets_files import q_to_graph

from creep_test_gui import Ui_CreepTestGUI



class CreepTestGUI(QMainWindow, Ui_CreepTestGUI):
    def __init__(self):
        super().__init__()
        self.setupUi(self)\
            
            
        ################################HINT TYPE###################################################
        
        
        self.textbox_offset_1.setPlaceholderTex("Enter +-480mA")
        self.textbox_offset_2.setPlaceholderTex("Enter +-480mA")
        
        self.textbox_direction1_1.setPlaceholderText("Direction 1")
        self.textbox_direction2_1.setPlaceholderText("Direction 2")
        self.textbox_direction1_2.setPlaceholderText("Direction 1")
        self.textbox_direction2_2.setPlaceholderText("Direction 2")
        
        self.textbox_vector_time_1.setPlaceholderText("for vector 1")
        self.textbox_vector_time_2.setPlaceholderText("for vector 2")
        
        self.textbox_sampling_rate.setPlaceholderText("specify the fast sampling rate!")
        self.textbox_sampling_time.setPlaceholderText("counting from 0s")
        
        self.textbox_standard_sampling_rate.setPlaceholderText("10000")
        #######################################################################################









