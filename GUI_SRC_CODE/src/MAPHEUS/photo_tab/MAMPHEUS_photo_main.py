import numpy as np
from PySide6 import QtGui
from PySide6.QtCore import QFileInfo, Qt
from PySide6.QtWidgets import *
import subprocess

from .MAMPHEUS_photo_tab import Ui_analyse_Window
from . import bit_to_jpg as convert_backend


from pathlib import Path
import os




class ConvertBinToJPG(QMainWindow, Ui_analyse_Window):

    def __init__(self):
        super().__init__()
        self.setupUi(self)
        #run
        self.run()
    
    def run(self):
        self._init_system_settings()
        
    def _init_system_settings(self):
        """ Find the relative path of the file """
        self.project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.choose_bin_file = self.csv_Button 
        self.choose_bin_file.clicked.connect(self.open_bin_file)
        
        
    def open_bin_file(self):
        bin_path, _ = QFileDialog.getOpenFileName(
            self, "Open .bin", "", "Binary files (*.bin *.BIN)"
        )
        if not bin_path:
            return

        #out directory 
        out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "converted")
        os.makedirs(out_dir, exist_ok=True)
        
        res = convert_backend.start_convert_one(bin_path, out_dir)
        if not res:
            return  # start_convert_one already prints the reason (e.g. wrong file size)
        
        jpg_name = os.path.splitext(os.path.basename(bin_path))[0] + ".jpg"
        jpg_path = os.path.join(out_dir, jpg_name)

        pixmap = QtGui.QPixmap(jpg_path)
        self.image_Label.setPixmap(
            pixmap.scaled(
                self.image_Label.size(),
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation,
            )
        )
