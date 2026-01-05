"""
Main entry point for GUI_SRC_CODE.

Responsibilities:
#. Manage GUI-related classes and functions.
#. Handle live data plotting.
#. Provide USB transmission functionality.

"""
from PyQt5 import QtGui
from PyQt5 import *
from PyQt5.QtWidgets import *
import sys
import os
from pathlib import Path


# #fix cache problem with MATHPLOTLIB
os.environ['MPLCONFIGDIR'] = str(Path.home()) +"/.matplotlib/"
import multiprocessing
import sys
import packet_transmission as packet_transmission
import socket_GUI
from main_window_gui import Ui_MainWindow
from creep_test.creep_test_window_main import TabWindowCreepTest
from constant_shear_rate.constant_shear_rate_main import TabWindowConstSR

# Source - https://stackoverflow.com/a
# Posted by DamonJW, modified by community. See post 'Timeline' for change history
# Retrieved 2025-12-05, License - CC BY-SA 4.0

if sys.platform == "win32":
    import ctypes
    myappid = 'mycompany.myproduct.subproduct.version' # arbitrary string
    ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(myappid)


from PyQt5 import QtCore, QtGui

class FirstGUI(QMainWindow, Ui_MainWindow): 
    """
    Main graphical user interface for the Mini-Rheometer (MIR) application.

    Inherits from `QMainWindow` and `Ui_MainWindow` to set up the main window
    and connect UI signals to their respective slots.

    :param QMainWindow: Base Qt main window class.
    :type QMainWindow: class
    :param Ui_MainWindow: Generated UI class from Qt Designer.
    :type Ui_MainWindow: class
    """   
    def __init__(self):
        
        """
        Initialize the MainGUI window.

        - Sets up the UI.
        - Disables the experiment selection combo box initially.
        - Declares experiment windows without showing them.
        - Connects signals for electronic and experiment selection combo boxes.`
        """
        
        super().__init__()
        self.setupUi(self)
        
        ################for banner purposes
        # get absolute path of project root (folder containing 'src' and 'pics')
        self.project_root = os.path.dirname(os.path.abspath(__file__))
        
        banner_path = os.path.join(self.project_root, "pics", "logo_fzj.png")
        self.photo_label.setPixmap(QtGui.QPixmap(banner_path))
        pixmap = QtGui.QPixmap(banner_path)
        self.photo_label.setPixmap(
            pixmap.scaled(self.photo_label.size(), 
                        QtCore.Qt.KeepAspectRatio, 
                        QtCore.Qt.SmoothTransformation)
        )
        ####################################
        # disable experiment combobox initially
        self.choose_experiment_comboBox.setEnabled(False)
        self.choose_electronic_comboBox.setEnabled(False)

        # connect electronic combobox signal at init
        self.choose_electronic_comboBox.currentIndexChanged.connect(self.enable_choose_experiment)

        # connect experiment combobox signal at init
        self.choose_experiment_comboBox.activated.connect(self.choose_window)
        
        self.choose_com_comboBox.activated.connect(self.choose_com_port)
        
    def choose_com_port(self):
        """
        Handles user selection of the virtual COM port from the combo box.

        This method retrieves the currently selected COM port name from 
        `choose_com_comboBox` and updates the system configuration accordingly. 
        If the default placeholder option ("Pick Virtual COM Port for USB") is 
        selected, the electronic selection combo box is disabled. Otherwise, 
        the electronic selection is enabled, and the selected COM port name 
        is passed to `sockets_files.port_name_setter()` for further configuration.

        Notes
        -----
        - The combo box `choose_com_comboBox` must contain at least one valid 
          COM port entry.
        - The combo box `choose_electronic_comboBox` is enabled only when a 
          valid COM port is selected.
        - `sockets_files.port_name_setter` : Function used to store or configure 
        the selected COM port name.
        """
        
        choose_com_text = str(self.choose_com_comboBox.currentText())
        
        if choose_com_text == "Pick Virtual COM Port for USB":
            self.choose_electronic_comboBox.setEnabled(False)
        else:
            self.choose_electronic_comboBox.setEnabled(True)
            socket_GUI.sockets_files.port_name_setter(choose_com_text)
        

    
    def enable_choose_experiment(self, index):
        """
        Enable the experiment selection combo box if a valid electronic is selected.

        Sets the `ELECTRONICS_FLAG` in `packet_transmission` depending on the
        selected electronic.

        :param index: Current index of the electronic selection combo box.
        :type index: int
        """
        current_text = self.choose_electronic_comboBox.currentText()
        
        # If placeholder or empty, disable experiment combobox
        if current_text in ["Electronic 1", "Electronic 2"]:  
            self.choose_experiment_comboBox.setEnabled(True)
            
            ##set the flag for electronics
            if current_text == "Electronic 1":
                packet_transmission.set_electronics_flag(1)
            elif current_text == "Electronic 2":
                packet_transmission.set_electronics_flag(2)
                print("Elecrtonics flag", packet_transmission.ELECTRONICS_FLAG)
        else:
            self.choose_experiment_comboBox.setEnabled(False)
            self.choose_experiment_comboBox.setCurrentIndex(0)

    def choose_window(self, index):
        """
        Open the appropriate experiment window based on experiment selection.

        - Puts the selected mode into the shared queue `q_get_mir_mode`.
        - Opens the corresponding experiment window.
        - Closes the main window after opening the experiment window.

        :param index: Current index of the experiment selection combo box.
        :type index: int
        """
        mode = self.choose_experiment_comboBox.currentText()
        
        if mode == "Control shear rate":
            
            self.csr_window = TabWindowConstSR()    
            self.csr_window.showNormal()
            
        elif mode == "Creep experiment":
            self.ct_window = TabWindowCreepTest()
            self.ct_window.showNormal()
            
            
        self.close()



#TODO: f_R and k_b, K need to match the electronics used. 
def main():
    """
    Entry point for the application.

    Initializes the Qt application, sets the window icon depending on the platform,
    creates and shows the main GUI window, and starts the Qt event loop.

    :return: None
    """
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
    
