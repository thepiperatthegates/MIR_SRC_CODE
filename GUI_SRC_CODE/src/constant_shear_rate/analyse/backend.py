import numpy as np
from PySide6 import QtGui
from PySide6.QtCore import QFileInfo, Qt
from PySide6.QtWidgets import *

from .analyse_Window import Ui_analyse_Window
from .calculation import AnalyseCalculationMixin
from serial_comm import device_state

from scipy.signal import savgol_filter
import pandas

from pathlib import Path
import os

os.environ['MPLCONFIGDIR'] = str(Path.home()) + "/.matplotlib/"

import matplotlib.pyplot as plt

plt.rcParams.update({
    'font.size': 14,
    # white figure background so the canvas reads as a distinct widget instead
    # of blending into the grey window (a border is added on the canvas below)
    'figure.facecolor': '#ffffff',
    'axes.facecolor': '#ffffff',
    'axes.edgecolor': '#bbbbbb',
    # visible against the white axes background (the old #dddddd was too faint)
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

data_4 = 0.0
data_6 = 0.0


# MATPLOTLIB CANVAS
class MatplotlibCanvas(FigureCanvasQTAgg):
    def __init__(self, parent=None, width=14, height=14, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        super().__init__(fig)


class AnalyseWindow(QMainWindow, Ui_analyse_Window, AnalyseCalculationMixin):
    def __init__(self) -> None:
        super().__init__()

        #------------------ init dir ------------------
        self.setupUi(self)

        project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

        # construct icon path
        save_icon_path = os.path.join(project_root, "pics", "save_icon.ico")
        self.save_Button.setIcon(QtGui.QIcon(save_icon_path))

        refresh_icon_path = os.path.join(project_root, "pics", "refresh_icon.ico")
        self.refresh_Button.setIcon(QtGui.QIcon(refresh_icon_path))

        # ---- make the top toolbar row uniform ----
        action_button_style = "background-color: rgb(85, 170, 255);"
        self.refresh_Button.setStyleSheet(action_button_style)
        self.csv_Button.setStyleSheet(action_button_style)
        self.save_Button.setStyleSheet(action_button_style)

        top_row_height = 28
        for widget in (
            self.refresh_Button,
            self.csv_Button,
            self.save_Button,
            self.data_show_comboBox,
            self.textbox_offset1,
            self.textbox_offset2,
        ):
            widget.setFixedHeight(top_row_height)

        self.horizontalLayout_2.setSpacing(12)

        self.header ="time[s];voltage_1[V];voltage_2[V];current_1[mA];current_2[mA];phase_magnet[deg];phase_field[deg];phase_diff[deg];angular_vel[rad/s];torque[N/m];shear_rate[1/s];shear_stress[Pa];Viscosity[Pa*s];offset_1[mA];offset_2[mA];fr0[Nm];fr1[Nm *s/rad]"

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

        self.worker_get_fr_coefficient = device_state.fRCoefficients()
        self.fr0 = self.worker_get_fr_coefficient.fr0
        self.fr1 = self.worker_get_fr_coefficient.fr1

        self.label_fr.setText(f"f<sub>r0</sub> = {self.fr0}&nbsp;&nbsp;&nbsp;"
                              f"f<sub>r1</sub> = {self.fr1}&nbsp;&nbsp;&nbsp;")

        self.COIL_CONSTANT = device_state.COIL_CONSTANT  # in T / A
        self.DIPOLE_MOMENT = device_state.DIPOLE_MOMENT  # in A m^2
        self.CALIBRATION_FACTOR = self.worker_get_fr_coefficient.CALIBRATION_FACTOR  # torque calibration no units (K)

        self.worker_get_offset = device_state.TxData()

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

        self.table_Widget.setColumnCount(13)

        # let columns share the available width instead of a fixed pixel scroll-fest
        self.table_Widget.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)

        # alternating row shading makes a 13-column table much easier to scan
        self.table_Widget.setAlternatingRowColors(True)
        self.table_Widget.setStyleSheet("QTableWidget { alternate-background-color: #ececec; }")

        # visually group the offset controls without touching the Designer layout tree
        self.layoutWidget.setStyleSheet(
            "QWidget#layoutWidget { border: 1px solid #bbbbbb; border-radius: 4px; padding: 4px; }"
        )

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
        self.mpl_toolbar = NavigationToolbar2QT(self.canvas, self.centralwidget)
        self.canvas_frame.hide()
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
            self.canvas_frame.hide()
            self.data_mode_function()

        elif mode in diagram_map:
            # DRAW THE GRAPH
            self.table_Widget.hide()
            self.canvas_frame.show()
            diagram_map[mode]()

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
                item.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
                self.table_Widget.setItem(row + 1, col, item)  # shift by 1

        #################################SET DATA FOR MEAN VALUE OF THE FIRST ROW#######################

        # first 7 columns with "-"
        for col in range(7):
            item = QTableWidgetItem("-")
            item.setBackground(QtGui.QColor(255, 0, 0))
            item.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
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
            item.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            self.table_Widget.setItem(0, i, item)

        ##############################################################################################

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

        # enable minor ticks first so the minor grid actually has locations to draw at
        self.canvas.axes.minorticks_on()
        self.canvas.axes.grid(True, which='major', linestyle='--', linewidth=0.6,
                              color='#b0b0b0', alpha=0.8)
        self.canvas.axes.grid(True, which='minor', linestyle=':', linewidth=0.4,
                              color='#cccccc', alpha=0.6)
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
    project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

    # construct icon path
    fzj_icon_path = os.path.join(project_root, "pics", "fzj.ico")
    app3.setWindowIcon(QtGui.QIcon(fzj_icon_path))
    window3 = AnalyseWindow()
    window3.show()
    app3.exec()


if __name__ == '__main__':
    main_3()
