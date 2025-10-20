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
        self.setupUi(self)









