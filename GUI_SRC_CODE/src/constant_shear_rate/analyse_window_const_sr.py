import numpy as np
from PyQt5 import QtGui
from PyQt5.QtCore import QFileInfo
from PyQt5.QtWidgets import *

from .analyse_Window import Ui_analyse_Window
import packet_transmission

from scipy.signal import savgol_filter
import pandas

from pathlib import Path
import os

os.environ['MPLCONFIGDIR'] = str(Path.home()) + "/.matplotlib/"

import matplotlib.pyplot as plt

plt.rcParams.update({'font.size': 14})

import matplotlib as mpl

mpl.use('Qt5Agg')

from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT

data_4 = 0.0
data_6 = 0.0


# MATPLOTLIB CANVAS
class MatplotlibCanvas(FigureCanvasQTAgg):
    def __init__(self, parent=None, width=14, height=14, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        super().__init__(fig)


class AnalyseWindow(QMainWindow, Ui_analyse_Window):
    def __init__(self) -> None:
        super().__init__()

        #------------------ init dir ------------------
        self.setupUi(self)

        project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

        # construct icon path
        save_icon_path = os.path.join(project_root, "pics", "save_icon.ico")
        self.save_Button.setIcon(QtGui.QIcon(save_icon_path))
        
        refresh_icon_path = os.path.join(project_root, "pics", "refresh_icon.ico")
        self.refresh_Button.setIcon(QtGui.QIcon(refresh_icon_path))
       
        
        self.header = "time[s];voltage_1[V];voltage_2[V];current_1[mA];current_2[mA];phase_magnet[deg];phase_field[deg];phase_diff[deg];angular_vel[rad/s];torque[N/m];shear_rate[1/s];shear_stress[Pa];Viscosity[Pa*s];offset_1[mA];offset_2[mA];fr0[Nm];fr1[Nm *s/rad]"

        #------------------ init variables for this class ------------------
        self.data = np.array([])
        self.analyse_filename = ' '
        self.final_data_to_save = np.array([])
        self.final_data_to_show = np.array([])
        self.num_rows = None
        self.num_column = None
        self.total_torque = None
        

        #------------------ calculation constant ------------------

        #------------------ inherited from another process ------------------

        self.worker_get_fr_coefficient = packet_transmission.fRCoefficients()
        self.fr0 = self.worker_get_fr_coefficient.fr0
        self.fr1 = self.worker_get_fr_coefficient.fr1

        self.label_fr.setText(f"f<sub>r0</sub> = {self.fr0}&nbsp;&nbsp;&nbsp;"
                              f"f<sub>r1</sub> = {self.fr1}&nbsp;&nbsp;&nbsp;")

        self.COIL_CONSTANT = packet_transmission.COIL_CONSTANT  # in T / A
        self.DIPOLE_MOMENT = packet_transmission.DIPOLE_MOMENT  # in A m^2
        self.CALIBRATION_FACTOR = self.worker_get_fr_coefficient.CALIBRATION_FACTOR  # torque calibration no units (K)

        self.worker_get_offset = packet_transmission.TxData()

        self.offset_1 = float(self.worker_get_offset.data_4)
        self.offset_2 = float(self.worker_get_offset.data_6)

        # placeholder for the offsets input
        self.textbox_offset1.setText(str(self.offset_1))
        self.textbox_offset2.setText(str(self.offset_2))

        # ------------------ geometry constants ------------------
        self.C_SS = 11160103  # conversion factor to stress in Pa / Nm
        self.C_SR = 37.099  # conversion factor to shear rate in s^-1 / s^-1
        

        # ------------------variables to save ------------------
        # ready up variables
        self.time = None
        self.current_1 = None
        self.current_2 = None
        self.voltage_1 = None
        self.voltage_2 = None
        # ------------------------------------------------------

        #------------------ declare bunch of important variable stuffs ------------------
        self.angle_magnetic_field = None
        self.angle_magnet = None
        self.angle_magnetic_field_unwrapped = None
        self.angle_magnet_unwrapped = None
        self.angle_magnetic_field_degree = None
        self.angle_magnet_degree = None 
        self.phase_difference = None
        self.angular_velocity = None
        self.fr1on_moment = None
        self.magnitude_current = None
        self.shear_rate = None
        self.total_torque = None
        self.shear_stress = None
        self.viscosity = None
        
        self.phase_difference_degree_mean = None
        self.angular_velocity_mean = None
        self.total_torque_mean = None
        self.shear_rate_mean = None
        self.shear_stress_mean = None
        self.viscosity_mean = None
        # ------------------------------------------------------------------------------------------

        self.setWindowTitle("Data analyse")

        self.data_show_comboBox.setCurrentIndex(-1)

        self.table_Widget.setColumnWidth(0, 100)
        self.table_Widget.setColumnWidth(1, 100)
        self.table_Widget.setColumnWidth(2, 100)
        self.table_Widget.setColumnWidth(3, 100)
        self.table_Widget.setColumnWidth(4, 100)
        self.table_Widget.setColumnWidth(5, 200)
        self.table_Widget.setColumnWidth(6, 200)
        self.table_Widget.setColumnWidth(7, 200)
        self.table_Widget.setColumnWidth(8, 200)
        self.table_Widget.setColumnWidth(9, 200)
        self.table_Widget.setColumnWidth(10, 200)
        self.table_Widget.setColumnWidth(11, 200)
        self.table_Widget.setColumnWidth(12, 200)

        self.table_Widget.setColumnCount(13)

        self.canvas = MatplotlibCanvas(self)
        self.mlp_layout.addWidget(self.canvas)
        self.mpl_toolbar = NavigationToolbar2QT(self.canvas, self.centralwidget)
        self.canvas.hide()
        self.horizontalLayout.addWidget(self.mpl_toolbar)
        self.csv_Button.clicked.connect(self.find_filename_button_pressed)
        self.save_Button.clicked.connect(self.save_button_event)
        self.data_show_comboBox.setDisabled(True)
        self.save_Button.setDisabled(True)

        self.textbox_offset1.editingFinished.connect(self.save_offset1_event_textbox)
        self.textbox_offset2.editingFinished.connect(self.save_offset2_event_textbox)
        
        #refresh button event clicked
        self.refresh_Button.clicked.connect(self.refresh_event)
        #toggle button for radio button offsets
        self.take_Button.toggled.connect(self.refresh_event)

    def save_offset1_event_textbox(self):
        self.offset_1 = float(self.textbox_offset1.text())

    def save_offset2_event_textbox(self):
        self.offset_2 = float(self.textbox_offset2.text())

    def find_filename_button_pressed(self):
        self.analyse_filename = QFileDialog.getOpenFileName(filter="csv (*.csv)")[0]
        self.label_file.setText(QFileInfo(self.analyse_filename).fileName())
        self.data_show_comboBox.setDisabled(False)
        self.data_show_comboBox.setCurrentText("Data table")
        self.choose_option()
        self.data_show_comboBox.activated.connect(self.choose_option)

    def choose_option(self):
        #------------------ read csv files ------------------
        # Notes: the legacy codes has the files without headers, recent updates has a header for the variables 
        with open(self.analyse_filename, 'r', encoding='utf-8-sig') as check:
            # check the first line of csv files
            first_line = check.readline().strip()
            
            
        # check if the first line is a digit (or the first line contains digit with negative number)
        if first_line[0].isdigit() or first_line[0] == '-':
            skip_check = 0
            print("Legacy code")
        else:
            skip_check = 1
            print("New code")

        # Load selected file
        self.data = np.genfromtxt(
            self.analyse_filename, 
            delimiter=";", 
            skip_header=skip_check, 
            encoding='utf-8-sig'
        )
            
        mode = self.data_show_comboBox.currentText()
        self.num_rows, self.num_column = self.data.shape

        self.data_calculation_function()
        
        diagram_map = {
            "Currents diagram" : self.draw_current_diagrams,
            "Voltage diagram" : self.draw_voltage_diagrams,
            "Phase diagram" : self.draw_phase_diagram,
            "Phase difference diagram" : self.draw_phase_difference_diagram,
            "Angular velocity diagram" :  self.draw_angular_velocity_diagram,
            "Torque diagram" :          self.draw_torque_diagram,
            "Shear rate diagram":       self.draw_shear_rate_diagram,
            "Shear stress diagram":     self.draw_shear_stress_diagram,
            "Viscosity diagram":   self.draw_viscosity_diagram,
        }
        
        
        self.offsets_update()
        # first mode
        if mode == "Data table":
            # HIDE THE TABLE WIDGET
            self.table_Widget.show()
            self.canvas.hide()
            self.data_mode_function()
            
        elif mode in diagram_map:
            # DRAW THE GRAPH
            self.table_Widget.hide()
            self.canvas.show()
            diagram_map[mode]()
            
            

    def data_calculation_function(self):
        if self.num_column == 5:

            self.determine_offsets_option()
            self.calculate_functions()
            self.calculate_all_mean()

            # remove last two columns
            self.final_data_to_show = self.final_data_to_save[:, :-4]

            self.take_Button.setDisabled(False)
            self.save_Button.setDisabled(False)

        elif self.num_column == 17:

            # take all the data up to column 13
            self.final_data_to_show = self.data[:, :-4]
            # take all the data 2 from the last column
            self.coefficient_saved = self.data[:, -4:]
            self.calculate_all_mean_after_save()
            self.reference_var_for_saved_data()

            self.take_Button.setDisabled(True)
            self.save_Button.setDisabled(True)
            
    def refresh_event(self):
        self.choose_option()

    def data_mode_function(self):

        # set row and column count
        self.table_Widget.setRowCount(self.final_data_to_show.shape[0] + 1)
        self.table_Widget.setColumnCount(self.final_data_to_show.shape[1])

        # insert mean row at the top
        self.table_Widget.insertRow(0)

        # fill the table starting from row 1
        for row in range(self.final_data_to_show.shape[0]):
            for col in range(self.final_data_to_show.shape[1]):
                item = QTableWidgetItem(str(self.final_data_to_show[row, col]))
                self.table_Widget.setItem(row + 1, col, item)  # shift by 1

        #################################SET DATA FOR MEAN VALUE OF THE FIRST ROW#######################

        # first 7 columns with "-"
        for col in range(7):
            item = QTableWidgetItem("-")
            item.setBackground(QtGui.QColor(255, 0, 0))
            self.table_Widget.setItem(0, col, item)

        # remaining columns with actual mean values
        values = [
            self.phase_difference_degree_mean,
            self.angular_velocity_mean,
            self.total_torque_mean,
            self.shear_rate_mean,
            self.shear_stress_mean,
            self.viscosity_mean,
        ]

        # colour the row with red
        for i, val in enumerate(values, start=7):
            item = QTableWidgetItem(str(val))
            item.setBackground(QtGui.QColor(255, 0, 0))
            self.table_Widget.setItem(0, i, item)

        ##############################################################################################

    def calculate_functions(self):
        #############################################################################
        self.time = np.zeros((self.num_rows, 1))
        self.current_1 = np.zeros((self.num_rows, 1))
        self.current_2 = np.zeros((self.num_rows, 1))
        self.voltage_1 = np.zeros((self.num_rows, 1))
        self.voltage_2 = np.zeros((self.num_rows, 1))
        self.angle_magnetic_field = np.zeros((self.num_rows, 1))
        self.angle_magnetic_field_unwrapped = np.zeros((self.num_rows, 1))
        self.angle_magnetic_field_degree = np.zeros((self.num_rows, 1))
        self.angle_magnet = np.zeros((self.num_rows, 1))
        self.angle_magnet_unwrapped = np.zeros((self.num_rows, 1))
        self.angle_magnet_degree = np.zeros((self.num_rows, 1))
        self.phase_difference = np.zeros((self.num_rows, 1))
        self.angular_velocity = np.zeros((self.num_rows, 1))
        self.shear_rate = np.zeros((self.num_rows, 1))
        self.fr1on_moment = np.zeros((self.num_rows, 1))
        self.total_torque = np.zeros((self.num_rows, 1))
        self.magnitude_current = np.zeros((self.num_rows, 1))
        ###############################################################################

        # declare variables to read from the files (already given)
        self.time = self.data[:, 0]
        self.voltage_1 = self.data[:, 1]
        self.voltage_2 = self.data[:, 2]
        
        print("self.offset_1, self.offset_2",  self.offset_1)
        print(self.offset_2)
        self.data[:, 3] -= self.offset_1
        self.data[:, 4] -= self.offset_2

        self.current_1 = self.data[:, 3]
        self.current_2 = self.data[:, 4]

        self.label_fr.setText(f"f<sub>r0</sub> = {self.fr0}&nbsp;&nbsp;&nbsp;"
                              f"f<sub>r1</sub> = {self.fr1}&nbsp;&nbsp;&nbsp;")

        self.calculate_angle()
        self.calculate_magnitude_current()
        # calculate rotation velocity
        self.calculate_shear_rate()

        # calculate friction moment from shear rate
        self.calculate_friction_moment()
        self.calculate_shear_stress()
        self.calculate_viscosity()

        amf = self.angle_magnetic_field_degree.reshape(-1, 1)
        amag = self.angle_magnet_degree.reshape(-1, 1)
        pd = self.phase_difference_degree.reshape(-1, 1)
        angv = self.angular_velocity.reshape(-1, 1)
        trq = self.total_torque.reshape(-1, 1)
        sr1 = self.shear_rate.reshape(-1, 1)
        ss2 = self.shear_stress.reshape(-1, 1)
        vis = self.viscosity.reshape(-1, 1)

        # get the fr from packet tranmision
        self.fr0 = self.worker_get_fr_coefficient.fr0
        self.fr1 = self.worker_get_fr_coefficient.fr1

        # change text box for visibility
        self.textbox_offset1.setText(str(self.offset_1))
        self.textbox_offset2.setText(str(object=self.offset_2))

        N = self.angle_magnetic_field.shape[0]  # number of rows

        # make empty string columns
        self.off1_to_be_saved = np.full((N, 1), "", dtype=object)
        self.off2_to_be_saved = np.full((N, 1), "", dtype=object)
        self.fr0_to_be_saved = np.full((N, 1), "", dtype=object)
        self.fr1_to_be_saved = np.full((N, 1), "", dtype=object)

        # put user input only in the first row
        self.off1_to_be_saved[0, 0] = float(self.offset_1)
        self.off2_to_be_saved[0, 0] = float(self.offset_2)
        self.fr0_to_be_saved[0, 0] = float(self.fr0)
        self.fr1_to_be_saved[0, 0] = float(self.fr1)

        self.final_data_to_save = np.hstack((
            self.data,
            amf,
            amag,
            pd,
            angv,
            trq,
            sr1,
            ss2,
            vis,
            self.off1_to_be_saved,
            self.off2_to_be_saved,
            self.fr0_to_be_saved,
            self.fr1_to_be_saved
        ))

    def reference_var_for_saved_data(self):
        #############################################################################
        self.time = np.zeros((self.num_rows, 1))
        self.current_1 = np.zeros((self.num_rows, 1))
        self.current_2 = np.zeros((self.num_rows, 1))
        self.voltage_1 = np.zeros((self.num_rows, 1))
        self.voltage_2 = np.zeros((self.num_rows, 1))
        self.angle_magnetic_field = np.zeros((self.num_rows, 1))
        self.angle_magnet = np.zeros((self.num_rows, 1))
        self.phase_difference = np.zeros((self.num_rows, 1))
        self.angular_velocity = np.zeros((self.num_rows, 1))
        self.shear_rate = np.zeros((self.num_rows, 1))
        self.fr1on_moment = np.zeros((self.num_rows, 1))
        self.total_torque = np.zeros((self.num_rows, 1))
        self.magnitude_current = np.zeros((self.num_rows, 1))
        ###############################################################################

        # declare variables to read from the files (already given)
        self.time = self.final_data_to_show[:, 0]
        self.voltage_1 = self.final_data_to_show[:, 1]
        self.voltage_2 = self.final_data_to_show[:, 2]
        self.current_1 = self.final_data_to_show[:, 3]
        self.current_2 = self.final_data_to_show[:, 4]
        self.magnitude_current = np.hypot(self.current_1 , self.current_2)
        self.angle_magnetic_field = self.final_data_to_show[:, 5]
        self.angle_magnet = self.final_data_to_show[:, 6]
        self.phase_difference = self.final_data_to_show[:, 7]
        self.angular_velocity = self.final_data_to_show[:, 8]
        self.total_torque = self.final_data_to_show[:, 9]
        self.shear_rate = self.final_data_to_show[:, 10]
        self.shear_stress = self.final_data_to_show[:, 11]
        self.viscosity = self.final_data_to_show[:, 12]

        self.offset_1 = self.coefficient_saved[0, 0]
        self.offset_2 = self.coefficient_saved[0, 1]
        self.fr0 = self.coefficient_saved[0, 2]
        self.fr1 = self.coefficient_saved[0, 3]

        self.textbox_offset1.setText(str(self.offset_1))
        self.textbox_offset2.setText(str(object=self.offset_2))


    def offsets_update(self):
        """
        Updates the offsets from the backend and update the textboxes in the GUI

        Returns: none

        """
        #change text box for visibility
        self.textbox_offset1.setText(str(self.offset_1))
        self.textbox_offset2.setText(str(self.offset_2))


        print("Offset 1 from csv:", self.offset_1)  #[mA]
        print("Offset 2 from csv:", self.offset_2) #[mA]
        print("fr0 from csv:", self.fr0) #[Nm]
        print("fr1 from csv:", self.fr1)    #[Nm * s/rad]


    def save_button_event(self):

        self.save_Button.setEnabled(False)
        
        
        filename, _ = QFileDialog.getSaveFileName(parent=self, caption="Save File", directory="",
                                                  filter="CSV Files (*.csv)")
        if filename:
            try:
                # Ensure filename ends with .csv if the user didn't type it
                if not filename.endswith('.csv'):
                    filename += '.csv'

                df = pandas.DataFrame(self.final_data_to_save)
                header_list = self.header.split(';')
                
                # Added encoding for better compatibility
                df.to_csv(filename, sep=";", index=False, header=header_list, encoding='utf-8-sig')
                
                print(f"File successfully saved to: {filename}")
                
            except PermissionError:
                print("Error: The file is currently open in another program. Please close it and try again.")
            except Exception as e:
                print(f"An unexpected error occurred: {e}")
            
            
        self.save_Button.setEnabled(True)

    def calculate_angle(self):
        """
        Calculate magnet and magnetic field angles and their phase difference.

        This method computes the angles for the magnet and the magnetic field
        from the Hall voltage data stored in `self.data`.

        - The magnet angle is calculated using columns 2 and 3 (`self.data[:, 1]` and `self.data[:, 2]`).
        - The magnetic field angle is calculated using columns 4 and 5 (`self.data[:, 3]` and `self.data[:, 4]`).
        - Angles are unwrapped along axis 0 to remove discontinuities.
        - The phase difference between the magnetic field and the magnet is stored
        in `self.phase_difference`.

        :return: None
        """
        for row in range(self.num_rows):
            # angle from 2nd and 3rd columns (index 1 and 2)
            self.angle_magnet[row, 0] = np.arctan2(self.data[row, 2], self.data[row, 1])


            # angle from 4th and 5th columns (index 3 and 4)
            self.angle_magnetic_field[row, 0] = np.arctan2(self.data[row, 4], self.data[row, 3])
        # TODO: OFFSETS

        # calculate the angles
        
        self.angle_magnetic_field_degree = np.degrees(self.angle_magnetic_field) #[deg]
        self.angle_magnet_degree = np.degrees(self.angle_magnet) #[deg]
        
        self.angle_magnetic_field_unwrapped = np.unwrap(self.angle_magnetic_field, axis=0) #[rad]
        self.angle_magnet_unwrapped = np.unwrap(self.angle_magnet, axis=0) #[rad]
        
        self.phase_difference = self.angle_magnetic_field_unwrapped - self.angle_magnet_unwrapped # [rad]
        self.phase_difference_degree = np.degrees(self.phase_difference)  # [degree]

    def calculate_shear_rate(self):
        """
        Calculate the shear rate from the magnet angle signal.

        This method uses a Savitzky-Golay filter to smooth and differentiate
        the noisy angular position data (`self.angle_magnet`) with respect
        to time (`self.time`). The first derivative of the angle signal gives
        the angular velocity in [rad/s]. The shear rate is then obtained by
        scaling the angular velocity with the shear rate constant `C_SR`.

        Steps:
            1. Apply Savitzky-Golay filter to estimate angular velocity.
            2. Compute shear rate as angulalabelr_velocity * shear rate coefficient [C_SR].

        Updates Attributes:
            self.angular_velocity : np.ndarray
                Estimated angular velocity of the magnet [rad/s].
            self.shear_rate : np.ndarray
                Calculated shear rate [1/s].

        Notes:
            - The smoothness depends on the chosen `window_length` and `polyorder`.
            - `delta` is set as the mean time step from `self.time`.
        """
        self.angular_velocity = savgol_filter(
            self.angle_magnet_unwrapped[:, 0],  
            window_length=101,  # try 51, 101
            polyorder=2,  # 2 or 3 
            deriv=1,  # first derivative
            delta=np.mean(np.diff(self.time))  # time step
        )  # [rad /s]

        self.shear_rate = self.angular_velocity * self.C_SR  # [1 / s]

    def calculate_friction_moment(self):
        self.friction_moment = self.angular_velocity * self.fr1 + self.fr0  # - y_0        # [Nm]
        
    def calculate_magnitude_current(self):
        self.magnitude_current = np.hypot(self.current_1, self.current_2) # [mA]

    def calculate_shear_stress(self):

        self.total_torque = (
                self.CALIBRATION_FACTOR  # dimensionless
                * self.DIPOLE_MOMENT  # [A·m²]
                * self.COIL_CONSTANT  # [T/A]
                * (self.magnitude_current / 1000)  # mA → A
                * np.sin(self.phase_difference[:, 0])  # dimensionless
                - self.friction_moment  # [Nm]
        )
        self.shear_stress = self.total_torque * self.C_SS  # [Pa]

    def calculate_viscosity(self):
        self.viscosity = self.shear_stress / self.shear_rate  # [Pa * s]

    def calculate_all_mean(self):

        self.phase_difference_degree_mean = np.mean(self.phase_difference_degree)
        print(self.phase_difference_degree_mean)
        self.angular_velocity_mean = np.mean(self.angular_velocity)
        self.total_torque_mean = np.mean(self.total_torque)
        self.shear_rate_mean = np.mean(self.shear_rate)
        self.shear_stress_mean = np.mean(self.shear_stress)
        self.viscosity_mean = np.mean(self.viscosity)

    def calculate_all_mean_after_save(self):

        self.phase_difference_degree_mean = np.mean(self.data[:, 7])
        self.angular_velocity_mean = np.mean(self.data[:, 8])
        self.total_torque_mean = np.mean(self.data[:, 9])
        self.shear_rate_mean = np.mean(self.data[:, 10])
        self.shear_stress_mean = np.mean(self.data[:, 11])
        self.viscosity_mean = np.mean(self.data[:, 12])
        
    def determine_offsets_option(self):
        
        # take from the main tab
        if self.take_Button.isChecked():
            
            self.offset_1 = float(self.worker_get_offset.data_4)
            self.offset_2 = float(self.worker_get_offset.data_6)
            self.textbox_offset1.setDisabled(True)
            self.textbox_offset2.setDisabled(True)
        else:
            # take from the textbox fields
            try:
                self.offset_1 = float(self.textbox_offset1.text())
                self.offset_2 = float(self.textbox_offset2.text())
                self.textbox_offset1.setDisabled(False)
                self.textbox_offset2.setDisabled(False)
            except ValueError:
                print("Invalid offset inputs")
                
                
    def template_draw_diagram(self, title, ylabel, lines, xlabel=r"Time / s"):
        
        self.canvas.axes.cla()
        self.canvas.axes.set_title(title, fontsize=20)
        self.canvas.axes.set_ylabel(ylabel, fontsize=20)
        self.canvas.axes.set_xlabel(xlabel, fontsize=20)
        
        #iterate through the lines mapping variables
        for line in lines:
            plot_line, = self.canvas.axes.plot(self.time, line["y"], color=line.get("color"))
            #check if there is label attached to it
            if "label" in line:
                plot_line.set_label(line["label"])
                
                
        # Only show legend if at least one label exists
        if any("label" in line for line in lines):
            self.canvas.axes.legend(loc='upper right', bbox_to_anchor=(1, 1), fontsize=15)

        self.canvas.axes.grid(True, which='both', linestyle='--', linewidth=0.5)
        self.canvas.axes.minorticks_on()
        self.canvas.draw()
                
        
        
    def draw_current_diagrams(self):
        title = r"Current Sensors"
        ylabel = r"Current / mA"
        lines = [
            {"y": self.current_1, "label" : r"Current 1 $I_1$", "color": "g"},
            {"y": self.current_2, "label" : r"Current_2 $I_2$", "color": "#FFB6C1"},
            {"y": self.magnitude_current, "label":r"Magnitude $\hat I$", "color": "r"},
        ]
        
        self.template_draw_diagram(title, ylabel, lines)
        

    def draw_voltage_diagrams(self):
        title = r"Voltage Sensors"
        ylabel = r"Voltage / V"
        lines = [
            {"y": self.voltage_1, "label": r"Hall sensor 1 $U_1$", "color": "#890304"},
            {"y": self.voltage_2, "label": r"Hall sensor 2 $U_2$", "color": "#00113a"},
        ]

        self.template_draw_diagram(title, ylabel, lines)

    def draw_phase_diagram(self):
        title = "Phase diagram"
        ylabel = r"Angle $\phi$ / °"
        lines = [
            {"y": np.degrees(self.angle_magnetic_field.reshape(-1, 1)), "label": r"$\phi_B$", "color": "#890304"},
            {"y": np.degrees(self.angle_magnet.reshape(-1, 1)), "label": r"$\phi_m$", "color": "#00113a"},
            ]
        self.template_draw_diagram(title, ylabel, lines)
        
        
    def draw_phase_difference_diagram(self):
        
        title = "Phase difference diagram"
        ylabel = r"Angle $\phi$ / °"
        lines = [
            {"y": self.phase_difference_degree, "label": r"$\Delta\phi$", "color": "#7294D4"}
        ]
        self.template_draw_diagram(title, ylabel, lines)

    def draw_angular_velocity_diagram(self):
        title = "Angular velocity diagram"
        ylabel = r"Angular velocity $\omega$ / rad$s^{-1}$"
        lines = [
            {"y": self.angular_velocity, "color": "red"},
        ]

        self.template_draw_diagram(title, ylabel, lines)

    def draw_torque_diagram(self):
        title = "Torque diagram"
        ylabel = r"Torque $T$ / Nm"
        lines = [
            {"y": self.total_torque, "color": "red"},
        ]

        self.template_draw_diagram(title, ylabel, lines)

    def draw_shear_rate_diagram(self):
        title = "Shear rate diagram"
        ylabel = r"Shear rate $\dot\gamma$ / $s^{-1}$"
        lines = [
            {"y": self.shear_rate, "color": "red"},
        ]

        self.template_draw_diagram(title, ylabel, lines)


    def draw_shear_stress_diagram(self):
        title = "Shear stress diagram"
        ylabel = r"Shear stress $\tau$ / Pa"
        lines = [
            {"y": self.shear_stress, "color": "red"},
        ]

        self.template_draw_diagram(title, ylabel, lines)


    def draw_viscosity_diagram(self):
        title = "Viscosity diagram"
        ylabel = r"Viscosity $\eta$"
        lines = [
            {"y": self.viscosity, "color": "red"},
        ]

        self.template_draw_diagram(title, ylabel, lines)

    def closeEvent(self, event):
        event.accept()

def main_3():
    app3 = QApplication([])

    # get absolute path of project root (folder containing 'src' and 'pics')
    project_root = os.path.dirname(os.path.abspath(__file__))

    # construct icon path
    fzj_icon_path = os.path.join(project_root, "pics", "fzj.ico")
    app3.setWindowIcon(QtGui.QIcon(fzj_icon_path))
    window3 = AnalyseWindow()
    window3.show()
    app3.exec_()


if __name__ == '__main__':
    main_3()