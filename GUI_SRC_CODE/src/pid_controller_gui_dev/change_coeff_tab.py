import sys
import os
from pathlib import Path

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5 import *
from PyQt5.QtCore import QThread, pyqtSignal, QMutex, QRegularExpression, QObject, QTimer
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QRegularExpressionValidator, QDoubleValidator

# #fix cache problem with MATHPLOTLIB
os.environ['MPLCONFIGDIR'] = str(Path.home()) +"/.matplotlib/"
import multiprocessing
import sys

import socket_GUI.sockets_files as sockets_files
from socket_GUI.sockets_files import q_to_graph

import packet_transmission as backend
from .change_coeff import Ui_Form


class SendPIDCoeff(QMainWindow, Ui_Form):
    
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        
        self._init_worker()
        self._connect_signals()
    
    def _init_worker(self):
        """Initialize all setter/getter flags from backend."""
        #for tx data
        self.worker_data_block = backend.TxData()
        #for transmitting thread
        self.worker_flag_send = backend.TxFlag()
        
    def _connect_signals(self):
        self.button_coeff_send.clicked.connect(self.send_data_event)
        self.button_coeff_send.clicked.connect(lambda: self.popout_window(7))
        
    def send_data_event(self):
        
        Kp = float(self.input_Kp.text())
        Ki = float(self.input_Ki.text())
        Kd = float(self.input_Kd.text())
        
        print('Kp:', Kp)
        print('Ki:', Ki)
        print('Kd', Kd)
        
        
        # --------------------- Update the Data Worker ---------------------
        self.worker_data_block.data_1 = Kp
    
        self.worker_data_block.data_2  = Ki
        self.worker_data_block.data_3 = Kd
        self.worker_data_block.data_8 = 3
        
        #for data 10
        self.worker_data_block.data_10 = 3 

    # ------- Popout window
    def popout_window(self, arg, calculate_final_fR = 0.0, k_b_1 = 0.0, k_b_2 = 0.0):

        msg = QMessageBox()
        
        text = backend.set_popout_text(arg, calculate_final_fR, k_b_1, k_b_2)
        msg.setText(text)

        msg.setIcon(QMessageBox.Question)

        msg.exec()
        
            
        



