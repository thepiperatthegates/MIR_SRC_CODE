"""Main entry point for the MiR (Mini-Rheometer) GUI application."""
from PyQt5 import QtGui, QtCore
from PyQt5 import *
from PyQt5.QtWidgets import *
import sys
import os
from pathlib import Path


# #fix cache problem with MATHPLOTLIB
os.environ['MPLCONFIGDIR'] = str(Path.home()) +"/.matplotlib/"
import multiprocessing
import sys
from serial_comm import device_state 
from main_window_gui import Ui_MainWindow # pyright: ignore[reportAttributeAccessIssue]
from creep_test.creep_test_window_main import TabWindowCreepTest
from constant_shear_rate.constant_shear_rate_main import TabWindowConstSR
from MAPHEUS.mapheus_main import TabWindowMAPHEUS

#import json for the GUI theme
from theme import LIGHT_THEME

if sys.platform == "win32":
    import ctypes
    myappid = 'mycompany.myproduct.subproduct.version' # arbitrary string
    ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(myappid)


class FirstGUI(QMainWindow, Ui_MainWindow):
    """Launcher window for selecting the electronics board and experiment type."""

    def __init__(self):
        
        super().__init__()
        self.setupUi(self)
        
        ################for banner purposes################################
        # get absolute path of project root (folder containing 'src' and 'pics')
        self.project_root = os.path.dirname(os.path.abspath(__file__))
        
        banner_path = os.path.join(self.project_root, "pics", "logo_fzj.png")
        self.photo_label.setPixmap(QtGui.QPixmap(banner_path))
        pixmap = QtGui.QPixmap(banner_path)
        self.photo_label.setPixmap(
            pixmap.scaled(self.photo_label.size(), 
                        QtCore.Qt.KeepAspectRatio,  # type: ignore
                        QtCore.Qt.SmoothTransformation) # type: ignore
        )
        ####################################################################
        # disable experiment combobox initially
        self.choose_experiment_comboBox.setEnabled(False)
        self.choose_electronic_comboBox.setEnabled(True)

        # connect electronic combobox signal at init
        self.choose_electronic_comboBox.currentIndexChanged.connect(self.enable_choose_experiment)

        # connect experiment combobox signal at init
        self.choose_experiment_comboBox.activated.connect(self.choose_window)
        
        

    
    def enable_choose_experiment(self, index):
        """Enable the experiment combo box and set the electronics flag when a board is selected."""
        current_text = self.choose_electronic_comboBox.currentText()
        
        # If placeholder or empty, disable experiment combobox
        if current_text in ["Electronic 1", "Electronic 2"]:  
            self.choose_experiment_comboBox.setEnabled(True)
            
            ##set the flag for electronics
            if current_text == "Electronic 1":
                device_state.set_electronics_flag(1)
            elif current_text == "Electronic 2":
                device_state.set_electronics_flag(2)
                print("Elecrtonics flag", device_state.ELECTRONICS_FLAG)
        else:
            self.choose_experiment_comboBox.setEnabled(False)
            self.choose_experiment_comboBox.setCurrentIndex(0)

    def choose_window(self, index):
        """Open the selected experiment window and close the launcher."""
        mode = self.choose_experiment_comboBox.currentText()
        
        if mode == "Control shear rate":
            
            self.csr_window = TabWindowConstSR()    
            self.csr_window.showNormal()
            
        elif mode == "Creep experiment":
            self.ct_window = TabWindowCreepTest()
            self.ct_window.showNormal()
        
        elif mode == "MAPHEUS":
            self.mapheus_window = TabWindowMAPHEUS()
            self.mapheus_window.showNormal()
            
        self.close()



#TODO: f_R and k_b, K need to match the electronics used. 
def main():
    """Initialize the Qt application, show the launcher window, and start the event loop."""
    app_main_window = QApplication(sys.argv)
    app_main_window.setStyleSheet(LIGHT_THEME)
    
    # get absolute path of project root (folder containing 'src' and 'pics')
    project_root = os.path.dirname(os.path.abspath(__file__))

    # construct icon path
    icon_path = os.path.join(project_root, "pics", "fzj.ico")
    
    # set window icon
    app_main_window.setWindowIcon(QtGui.QIcon(icon_path))
    
    first_window = FirstGUI()
    first_window.show()
    sys.exit(app_main_window.exec_())


if __name__ == '__main__':
    multiprocessing.freeze_support()
    
    main()
    
