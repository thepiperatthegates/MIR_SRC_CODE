"""
Main entry point for GUI_SRC_CODE.

Responsibilities:
#. Manage GUI-related classes and functions.
#. Handle live data plotting.
#. Provide USB transmission functionality.

"""
import sys
import os
import multiprocessing
from pathlib import Path

from PyQt5 import QtCore, QtGui
from PyQt5.QtWidgets import QApplication, QMainWindow

# fix cache problem with matplotlib
os.environ['MPLCONFIGDIR'] = str(Path.home()) + "/.matplotlib/"

import socket_GUI.device_state as device_state
import socket_GUI
from backend_main.main_window_gui import Ui_MainWindow # pyright: ignore[reportAttributeAccessIssue]
from creep_test.creep_test_window_main import TabWindowCreepTest
from constant_shear_rate.constant_shear_rate_main import TabWindowConstSR
from pid_controller_gui_dev.pid_output_main import TabWindowPID

if sys.platform == "win32":
    import ctypes
    myappid = 'fzj.mir.rheometer.1'
    ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(myappid)



class FirstGUI(QMainWindow, Ui_MainWindow):
    """Main window for the Mini-Rheometer (MIR) application."""

    def __init__(self):
        """Set up the UI, disable combo boxes, and connect signals."""
        
        super().__init__()
        self.setupUi(self)
        
        self.project_root = os.path.dirname(os.path.abspath(__file__))
        banner_path = os.path.join(self.project_root, "pics", "logo_fzj.png")
        pixmap = QtGui.QPixmap(banner_path)
        self.photo_label.setPixmap(
            pixmap.scaled(self.photo_label.size(),
                          QtCore.Qt.KeepAspectRatio,       # type: ignore
                          QtCore.Qt.SmoothTransformation)  # type: ignore
        )
        # disable experiment combobox initially
        self.choose_experiment_comboBox.setEnabled(False)
        self.choose_electronic_comboBox.setEnabled(False)

        # connect electronic combobox signal at init
        self.choose_electronic_comboBox.currentIndexChanged.connect(self.enable_choose_experiment)

        # connect experiment combobox signal at init
        self.choose_experiment_comboBox.activated.connect(self.choose_window)
        
        self.choose_com_comboBox.activated.connect(self.choose_com_port)
        
    def choose_com_port(self):
        """Enable electronics selection when a valid COM port is chosen, disable it otherwise."""
        
        choose_com_text = str(self.choose_com_comboBox.currentText())
        
        if choose_com_text == "Pick Virtual COM Port for USB":
            self.choose_electronic_comboBox.setEnabled(False)
        else:
            self.choose_electronic_comboBox.setEnabled(True)
            socket_GUI.serial_backend.port_name_setter(choose_com_text)
        

    
    def enable_choose_experiment(self, index):
        """Enable experiment selection and set the electronics flag when a valid electronic is chosen."""
        current_text = self.choose_electronic_comboBox.currentText()
        
        # If placeholder or empty, disable experiment combobox
        if current_text in ["Electronic 1", "Electronic 2"]:  
            self.choose_experiment_comboBox.setEnabled(True)
            
            ##set the flag for electronics
            if current_text == "Electronic 1":
                device_state.set_electronics_flag(1)
            elif current_text == "Electronic 2":
                device_state.set_electronics_flag(2)
        else:
            self.choose_experiment_comboBox.setEnabled(False)
            self.choose_experiment_comboBox.setCurrentIndex(0)

    def choose_window(self, index):
        """Open the selected experiment window and close the main window."""
        mode = self.choose_experiment_comboBox.currentText()
        
        if mode == "Control shear rate":
            
            self.csr_window = TabWindowConstSR()    
            self.csr_window.showNormal()
            
        elif mode == "Creep experiment":
            self.ct_window = TabWindowCreepTest()
            self.ct_window.showNormal()
            
            
        elif mode == "Pid-Controller (dev)":
            self.pid_window = TabWindowPID()
            self.pid_window.showNormal()
            
            
        self.close()



#TODO: f_R and k_b, K need to match the electronics used. 
def main():
    """Initialize the Qt application, show the main window, and start the event loop."""
    app_main_window = QApplication(sys.argv)
    
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
    
