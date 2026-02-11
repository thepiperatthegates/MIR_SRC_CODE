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
from .change_coeff import CoeffWindow


class SendPIDCoeff(QMainWindow, CoeffWindow):
    
    def __init__(self):
        super().__init__()
        self.setupUi
        
        self.button_send.clicked.connect(self.send_data_event)
                #for tx data
        self.worker_data_block = backend.TxData()
        #for transmitting thread
        self.worker_flag_send = backend.TxFlag()
    
    
    def send_data_event(self):
        
        
        ############################# THIS IS ALL A PLACEHOLDER ######################################
        self.worker_data_block.data_1 = 65534
    
        self.worker_data_block.data_2  = 10

        self.worker_data_block.data_8 = 0 
        
        #for data 10
        self.worker_data_block.data_10 = 1 
        self.worker_data_block.data_11 = self.textbox_shear_stress.text()
        
        
        ######################################################################################
        ##enabled stop rotating button 
        self.button_stop.setDisabled(False)
        
        ##send all data to microcontroller
        #activate flag
        self.worker_flag_send.flag_tx = True
        
        
        
    
    


