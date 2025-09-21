import numpy as np
from PyQt5 import QtCore, QtGui
from PyQt5 import *
from PyQt5.QtCore import QThread, pyqtSignal, QFileInfo
from PyQt5 import uic
from PyQt5.QtWidgets import *
import sys


from button_tekan_window import Ui_data_capture_Window
from analyse_Window import Ui_analyse_Window
import packet_transmission as packet_transmission

from scipy.signal import savgol_filter
import pandas 


#set the icon???????????
# import ctypes
# myappid = 'mycompany.myproduct.subproduct.version' # arbitrary string
# ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(myappid)

import os


import matplotlib.pyplot as plt
plt.rcParams.update({'font.size': 14})

import matplotlib as mpl

mpl.use('Qt5Agg')


from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT




data_4 =0.0
data_6 = 0.0


#MATPLOTLIB CANVAS
class MatplotlibCanvas(FigureCanvasQTAgg):
    def __init__(self, parent=None, width=14, height=14, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        super().__init__(fig)



class DataHandler(QThread):
    data_loaded_signal = QtCore.pyqtSignal(list)
    def __init__(self, filename,  interval=5000, parent = None):
        super(DataHandler, self).__init__(parent)
        self.filename = filename
        self.interval = interval
        self.running = True

    def run(self):
        while self.running:
            #TODO: try buat guna numpy array instead of list
            # data = np.loadtxt(self.analyse_filename, delimiter=';')

            with open(f"{str(self.filename)}.csv", "r") as file:
                data = []
                for row in file:
                    row_values = list(map(float, row.strip().split(';')))
                    data.append(row_values)
            self.data_loaded_signal.emit(data)
            self.msleep(self.interval)

    def stop(self):
        self.running = False
        
        
class SaveFile(QThread):
    
    def __init__(self, filename, parent=None):
        super().__init__(parent)
        self.filename = filename
        
    def run(self):
        filename_saving, _ = QFileDialog.getSaveFileName(self, "Save File", "", "CSV Files (*.csv)")
        if filename_saving:
            if not filename_saving.lower().endswith('.csv'):
                filename_saving += '.csv'
            data_read = np.loadtxt(f"{self.filename}.csv", delimiter=';')
            np.savetxt(filename_saving, data_read, delimiter=';')

class AcquisitionWindow(QMainWindow, Ui_data_capture_Window):
    def __init__(self, filename):
        super(AcquisitionWindow, self).__init__()
        self.filename = filename
        self.setupUi(self)

        self.setWindowTitle("Data acquisition")
        
        self.save_button.setIcon(QtGui.QIcon("../pics/fzj.png"))

        self.tableWidget.setColumnWidth(0, 250)
        self.tableWidget.setColumnWidth(1, 250)
        self.tableWidget.setColumnWidth(2, 250)
        self.tableWidget.setColumnWidth(3, 250)
        self.tableWidget. setColumnWidth(4, 250)
        self.stop_button.clicked.connect(self.stop_button_event)
        self.save_button.clicked.connect(self.save_button_event)


        self.tableWidget.setColumnCount(5)


        self.data_handling = DataHandler(self.filename, interval=5000) #100ms
        self.data_handling.start()
        self.data_handling.data_loaded_signal.connect(self.update_table)
        self.show()
    
    def save_button_event(self):
        
        self.save_File = SaveFile(self.filename)
        self.save_File.start()
        # filename_saving, _ = QFileDialog.getSaveFileName(self, "Save File", "", "CSV Files (*.csv)")
        # if filename_saving:
        #     if not filename_saving.lower().endswith('.csv'):
        #         filename_saving += '.csv'
        #     data_read = np.loadtxt(f"{self.filename}.csv", delimiter=';')
        #     np.savetxt(filename_saving, data_read, delimiter=';')
            
            
            
        
    def stop_button_event(self):
        self.data_handling.stop()

    def update_table(self, data):
        self.tableWidget.setRowCount(len(data))

        for row_number, row_data in enumerate(data):
            for col_number, cell in enumerate(row_data):
                # Create a QTableWidgetItem with the string representation of the float
                item = QTableWidgetItem(str(cell))
                self.tableWidget.setItem(row_number, col_number, item)
                
    def closeEvent(self, event):
        #stop threads
        self.data_handling.data_loaded_signal.disconnect(self.update_table)
        self.data_handling.stop()
        
        packet_transmission.running_time_event(5)
        print("tutup")

        #delete the dummy csv file once the window is closed
        try:
            os.remove("dummy.csv")
        except OSError as e:
            print(f"Error deleting file: {e}")
        event.accept()



def main_2(q_filename):
    filename = q_filename.get()
    app2 = QApplication([])
    app2.setWindowIcon(QtGui.QIcon('../pics/fzj.png'))
    window2 = AcquisitionWindow(filename)
    app2.exec_()


class AnalyseWindow(QMainWindow, Ui_analyse_Window):
    def __init__(self, parent= None) -> None:
        super().__init__(parent=None)
        
        self.setupUi(self)
        
        project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

        # construct icon path
        save_icon_path = os.path.join(project_root, "pics", "save_icon.png")
        self.save_Button.setIcon(QtGui.QIcon(save_icon_path))
            
        
    ############################init variables for this class###############################
        self.data = np.array([])
        self.analyse_filename = ' '
        self.final_data_to_save = np.array([])
        self.final_data_to_show = np.array([])
        self.num_rows = None
        self.num_column = None
        self.total_torque = None     
    #######################################################################################################

        
        #placeholder for the offsets input
        self.textbox_offset1.setText("0")  # default value 0
        self.textbox_offset2.setText("0")  # default value 0
    
        
        self.offset_1 =float(self.textbox_offset1.text())
        self.offset_2 =float(self.textbox_offset2.text())       
    
    
   ############calculation constant##############################################################
    
        self.FRICTION_COEFFICIENT = packet_transmission.f_R
        self.COIL_CONSTANT = 3.097e-3		# in T / A
        self.DIPOLE_MOMENT = 8.594e-3		# in A m^2
        self.CALIBRATION_FACTOR = 1		# torque calibration no units (K)

    ############geometry constants################################################################
        self.C_SS = 11160103			# conversion factor to stress in Pa / Nm
        self.C_SR = 37.099			# conversion factor to shear rate in s^-1 / s^-1
    #######################################################################################################
      
      
    ################################variables to save#####################################
        #ready up variables 
        self.time = None
        self.current_1 = None
        self.current_2 = None
        self.voltage_1 = None
        self.voltage_2 = None
        
        
        
        
        #declare bunch of important variable stuffs
        self.angle_magnetic_field = None
        self.angle_magnet = None
        self.phase_difference = None
        self.angular_velocity = None
        self.friction_moment = None
        self.magnitude_current = None
        self.shear_rate = None
        self.total_torque = None
        self.shear_stress = None
        self.viscosity = None
    #######################################################################################################
    
    #############################################################################
        self.phase_difference_mean = None
        self.angular_velocity_mean = None
        self.total_torque_mean = None
        self.shear_rate_mean = None
        self.shear_stress_mean = None
        self.viscosity_mean = None
        
     #############################################################################
        

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
        self.data_show_comboBox.setDisabled(True)
        self.save_Button.setDisabled(True)
        

    def find_filename_button_pressed(self):
        self.analyse_filename = QFileDialog.getOpenFileName(filter="csv (*.csv)")[0]
        self.label_file.setText(QFileInfo(self.analyse_filename).fileName())
        self.data_show_comboBox.setDisabled(False)
        self.data_show_comboBox.activated.connect(self.choose_option)
        
        
        

    def choose_option(self):
        
            
        mode = self.data_show_comboBox.currentText()
        
        
        self.data = pandas.read_csv(self.analyse_filename, sep=";", header=None).to_numpy()
        
        
        self.num_rows, self.num_column = self.data.shape


        #############################################################################
        self.time = np.zeros(( self.num_rows, 1))
        self.current_1 = np.zeros(( self.num_rows, 1))
        self.current_2 = np.zeros(( self.num_rows, 1))
        self.voltage_1 = np.zeros(( self.num_rows, 1))
        self.voltage_2 = np.zeros(( self.num_rows, 1))
        self.angle_magnetic_field = np.zeros(( self.num_rows, 1))
        self.angle_magnet = np.zeros(( self.num_rows, 1))
        self.phase_difference = np.zeros((self.num_rows, 1))
        self.angular_velocity = np.zeros((self.num_rows, 1))
        self.shear_rate =  np.zeros((self.num_rows, 1))
        self.friction_moment = np.zeros((self.num_rows, 1))
        self.total_torque =  np.zeros((self.num_rows, 1))
        self.magnitude_current = np.zeros((self.num_rows, 1))
        ###############################################################################
        
        
        
        
        
        
        #declare variables to read from the files (already given)
        self.time = self.data[:, 0]
        self.voltage_1 = self.data[:, 1]
        self.voltage_2 = self.data[:, 2]
        self.current_1 = self.data[:, 3]
        self.current_2 = self.data[:, 4]
        
        
        
        
         #call normalise function 
        self.normalise_voltage_event()
        self.calculate_angle()
        self.calculate_magnitude_current()
        # calculate rotation velocity
        self.calculate_shear_rate()
        
        #calculate friction moment from shear rate
        self.calculate_friction_moment()
        self.calculate_shear_stress()
        self.calculate_viscosity()

        
        
        amf = self.angle_magnetic_field.reshape(-1, 1)
        amag = self.angle_magnet.reshape(-1, 1)
        pd  = self.phase_difference.reshape(-1, 1)
        angv = self.angular_velocity.reshape(-1,1)
        trq = self.total_torque.reshape(-1, 1)
        sr1 = self.shear_rate.reshape(-1, 1)
        ss2 = self.shear_stress.reshape(-1, 1)
        vis = self.viscosity.reshape(-1, 1)
        
                
        N = self.angle_magnetic_field.shape[0]  # number of rows

        # make empty string columns
        off1 = np.full((N, 1), "", dtype=object)
        off2 = np.full((N, 1), "", dtype=object)

        # put user input only in the first row
        off1[0, 0] = str(self.offset_1)
        off2[0, 0] = str(self.offset_2)
        print(self.offset_1)
            
                        
                
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
            off1, 
            off2
        ))

        #find the number of rows and column AGAIN

        #first mode
        if mode == "Data table":
            #HIDE THE TABLE WIDGET 
            self.table_Widget.show()
            self.canvas.hide()
            self.data_mode_function()
            
        elif mode == "Currents diagram":
            #HIDE THE TABLE WIDGET 
            self.table_Widget.hide()
            self.canvas.show()
            self.draw_current_diagrams()
            
            
        elif mode == "Voltage diagram":
            self.table_Widget.hide()
            self.canvas.show()
            self.draw_voltage_diagrams()
            
        elif mode == "Phase diagram":
            self.table_Widget.hide()
            self.canvas.show()
            self.draw_phase_diagram()
            
        elif mode == "Angular velocity diagram":
            self.table_Widget.hide()
            self.canvas.show()
            self.draw_angular_velocity_diagram()
            
        elif mode == "Torque diagram":
            self.table_Widget.hide()
            self.canvas.show()
            self.draw_torque_diagram()
            
        elif mode == "Shear rate diagram":
            self.table_Widget.hide()
            self.canvas.show()
            self.draw_shear_rate_diagram()


        elif mode == "Shear stress diagram":
            self.table_Widget.hide()
            self.canvas.show()
            self.draw_shear_stress_diagram()

        elif mode == "Viscosity diagram":
            self.table_Widget.hide()
            self.canvas.show()
            self.draw_viscosity_diagram()
            
            
            


    def data_mode_function(self):
        if  self.num_column == 5:
            
            self.calculate_all_mean()

            # remove last two columns
            self.final_data_to_show = self.final_data_to_save[:, :-2]

            # set row and column count
            self.table_Widget.setRowCount(self.final_data_to_show.shape[0]+1)
            self.table_Widget.setColumnCount(self.final_data_to_show.shape[1])
            
            # insert mean row at the top
            self.table_Widget.insertRow(0)

            # fill the table starting from row 1
            for row in range(self.final_data_to_show.shape[0]):
                for col in range(self.final_data_to_show.shape[1]):
                    item = QTableWidgetItem(str(self.final_data_to_show[row, col]))
                    self.table_Widget.setItem(row + 1, col, item)  # shift by 1


                                
            self.save_Button.setDisabled(False)
            self.save_Button.clicked.connect(self.save_button_event)
            
            
        elif self.num_column == 15:

            self.calculate_all_mean_after_save()
            
            
            self.final_data_to_show = self.data[:, :-2]
            
            
            # set row and column count
            self.table_Widget.setRowCount(self.final_data_to_show.shape[0]+1)
            self.table_Widget.setColumnCount(self.final_data_to_show.shape[1])
            
            
            self.table_Widget.insertRow(0)

            for row in range(self.final_data_to_show.shape[0]):
                for col in range(self.final_data_to_show.shape[1]):
                    item = QTableWidgetItem(str(self.final_data_to_show[row, col]))
                    self.table_Widget.setItem(row + 1, col, item)  # shift by 1
                    
            self.save_Button.setDisabled(True)
            
            
        #################################SET DATA FOR MEAN VALUE OF THE FIRST ROW#######################
        
        
        # first 7 columns with "-"
        for col in range(7):
            item = QTableWidgetItem("-")
            item.setBackground(QtGui.QColor(255, 0, 0))
            self.table_Widget.setItem(0, col, item)

        # remaining columns with actual mean values
        values = [
            self.phase_difference_mean,
            self.angular_velocity_mean,
            self.total_torque_mean,
            self.shear_rate_mean,
            self.shear_stress_mean,
            self.viscosity_mean,
        ]

        for i, val in enumerate(values, start=7):
            item = QTableWidgetItem(str(val))
            item.setBackground(QtGui.QColor(255, 0, 0))
            self.table_Widget.setItem(0, i, item)


        
        ##############################################################################################
        

        
    
    def normalise_voltage_event(self):
        """
        Normalize Hall sensor voltage data.

        This method reads the Hall voltage data from `self.data` (columns 1 and 2),
        calculates the amplitude and zero offset, and then normalizes the voltage
        so that it is centered around 0 and scaled by its amplitude.

        After normalization, the voltage data in `self.data[:, 1]` and
        `self.data[:, 2]` will be in the range [-1, 1].

        :return: None
        """
        # #read column 2 and column 3 for the hall voltage
        amplitude_voltage_1 = (np.max(self.data[:, 1]) - np.min(self.data[:, 1]))/2.0
        zero_offset_voltage_1 = (np.max(self.data[:, 1]) + np.min(self.data[:, 1]))/2.0

        amplitude_voltage_2 = (np.max(self.data[:, 2]) - np.min(self.data[:, 2]))/2.0
        zero_offset_voltage_2 = (np.max(self.data[:, 2]) + np.min(self.data[:, 2]))/2.0

        self.data[:, 1] = (self.data[:, 1]- zero_offset_voltage_1 )/amplitude_voltage_1
        self.data[:, 2] = (self.data[:, 2]-  zero_offset_voltage_2 )/amplitude_voltage_2

    
    def save_button_event(self):
        filename, _ = QFileDialog.getSaveFileName(self, "Save File", "", "CSV Files (*.csv)")
        if filename:
            if not filename.lower().endswith('.csv'):
                filename += '.csv'
            
            # Save with pandas
            df = pandas.DataFrame(self.final_data_to_save)
            df.to_csv(filename, sep=";", index=False, header=False)
                
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

                    
                    
                    
        #calculate the angles 
        self.angle_magnetic_field = np.unwrap(self.angle_magnetic_field, axis=0)
        self.angle_magnet = np.unwrap(self.angle_magnet, axis=0)
        
        
        self.phase_difference = self.angle_magnetic_field - self.angle_magnet
    
    
    
    def calculate_shear_rate(self):

        self.angular_velocity = savgol_filter(
            self.angle_magnet[:, 0],      # your noisy angle signal
            window_length=400,            # try 51, 101, etc. depending on how smooth you want
            polyorder=3,                  # 2 or 3 works well
            deriv=1,                      # first derivative
            delta=np.mean(np.diff(self.time))  # time step
            )  # [rad /s] 
        
        
  
        
        self.shear_rate =  self.angular_velocity * self.C_SR # [1 / s]
        
        
        
    def calculate_friction_moment(self):
        self.friction_moment = self.CALIBRATION_FACTOR * self.angular_velocity * self.FRICTION_COEFFICIENT          # [Nm]
        
        
    def calculate_magnitude_current(self):
        self.offset_1 =float(self.textbox_offset1.text())
        self.offset_2 =float(self.textbox_offset2.text())   
        # print(self.offset_1)
        # print(self.offset_2)
        power_of_2 = np.power((self.current_1 - self.offset_1), 2) + np.power((self.current_2 - self.offset_2), 2)
        # power_of_2 = np.power(self.current_1, 2) + np.power(self.current_2, 2)
        self.magnitude_current = np.sqrt(power_of_2)
        
        
    def calculate_shear_stress(self):
        
        self.total_torque = (
        self.CALIBRATION_FACTOR                # dimensionslos
        * self.DIPOLE_MOMENT                    # [A·m²]
        * self.COIL_CONSTANT                    # [T/A]
        * self.magnitude_current / 1000         # mA -> A, [A]
        * np.sin(self.phase_difference[:, 0])   # dimensionless
        ) - self.friction_moment                # [Nm]
        
        
        
        self.shear_stress =  self.total_torque * self.C_SS	# [Pa]

    def calculate_viscosity(self):
        self.viscosity = self.shear_stress / self.shear_rate	# [Pa * s]
        
        
        
    def calculate_all_mean(self):
    
        self.phase_difference_mean =     np.mean(self.phase_difference)
        print(self.phase_difference_mean)
        self.angular_velocity_mean =     np.mean(self.angular_velocity)
        self.total_torque_mean =         np.mean(self.total_torque)
        self.shear_rate_mean =           np.mean(self.shear_rate)
        self.shear_stress_mean =           np.mean(self.shear_stress)
        self.viscosity_mean =            np.mean(self.viscosity)
        
        
    def calculate_all_mean_after_save(self):
    
        self.phase_difference_mean =     np.mean(self.data[:, 7])
        self.angular_velocity_mean =     np.mean(self.data[:, 8])
        self.total_torque_mean =         np.mean(self.data[:, 9])
        self.shear_rate_mean =           np.mean(self.data[:,10 ])
        self.shear_stress_mean =           np.mean(self.data[:, 11])
        self.viscosity_mean =            np.mean(self.data[:, 12])
        
        
        
                
    def draw_current_diagrams(self):

        #read current, 2nd and 3rd columns
        time =  self.time
        current1 =  self.current_1
        current2 =  self.current_2
        
        self.canvas.axes.cla()  #clear canvas
        self.canvas.axes.set_title(r"Current sensors", fontsize=20)
        self.canvas.axes.set_ylabel(r"Current / mA", fontsize=20)
        self.canvas.axes.set_xlabel(r"Time / s", fontsize=20)
        plot_1, =self.canvas.axes.plot(time, current1, color='g')
        plot_2, = self.canvas.axes.plot(time, current2, color='#FFB6C1')
        plot_3, = self.canvas.axes.plot(time, self.magnitude_current, color='r')
        plot_1.set_label(r"Current 1 $I_1$")
        plot_2.set_label(r"Current 2 $I_2$")
        plot_3.set_label(r"Magnitude $\hat I$")
        self.canvas.axes.legend(loc = 'upper right', bbox_to_anchor=(1,1), fontsize = 15)
        self.canvas.axes.grid(True, which='both', linestyle='--', linewidth=0.5)
        self.canvas.axes.minorticks_on()
        self.canvas.draw()

    def draw_voltage_diagrams(self):
        time = self.time
        voltage1    = self.voltage_1
        voltage2  = self.voltage_2


        self.canvas.axes.cla()  #clear canvas
        self.canvas.axes.set_title(r"Voltage sensors", fontsize=20)
        self.canvas.axes.set_ylabel(r"Voltage /  V", fontsize=20)
        self.canvas.axes.set_xlabel(r"Time / s", fontsize=20)
        plot_1, =self.canvas.axes.plot(time, voltage1, color='#890304')
        plot_2, = self.canvas.axes.plot(time, voltage2, color='#00113a')
        plot_1.set_label(r"Hall sensors 1 $U_1$")
        plot_2.set_label(r"Hall sensors 2 $U_2$")
        self.canvas.axes.legend(loc = 'upper right', bbox_to_anchor=(1,1))
        self.canvas.axes.grid(True, which='both', linestyle='--', linewidth=0.5)
        self.canvas.axes.minorticks_on()
        self.canvas.draw()
        
        
        
    def draw_phase_diagram(self):
        
        time = self.time
        phase_magnetic_field = self.angle_magnetic_field
        phase_magnet = self.angle_magnet
        phase_difference = self.phase_difference
        
        self.canvas.axes.cla()  #clear canvas
        self.canvas.axes.set_title("Phase diagramm")
        self.canvas.axes.set_ylabel(r"Angle $\phi$ / rad")
        self.canvas.axes.set_xlabel(r"Time / $s$")
        plot_1, =self.canvas.axes.plot(time, phase_magnetic_field, color='#890304')
        plot_2, = self.canvas.axes.plot(time, phase_magnet, color='#00113a') 
        plot_3, = self.canvas.axes.plot(time, phase_difference, color='#7294D4')
        plot_1.set_label(r"Angle of magnetic field $\phi_B$")
        plot_2.set_label(r"Angle of magnet $\phi_m$")
        plot_3.set_label(r"Phase difference $\Delta\phi$")
        self.canvas.axes.legend(loc = 'upper right', bbox_to_anchor=(1,1))
        self.canvas.axes.grid(True, which='both', linestyle='--', linewidth=0.5)
        self.canvas.axes.minorticks_on()
        self.canvas.draw()
        
        
    def draw_angular_velocity_diagram(self):
        
        self.canvas.axes.cla()  #clear canvas
        self.canvas.axes.set_title("Angular velocity diagram")
        self.canvas.axes.set_ylabel(r"Angular velocity $\omega$ / rad$s^{-1}$")
        self.canvas.axes.set_xlabel(r"Time / $s$")
        self.canvas.axes.plot(self.time,self.angular_velocity, color ='red')
        self.canvas.axes.grid(True, which='both', linestyle='--', linewidth=0.5)
        self.canvas.axes.minorticks_on()
        self.canvas.draw()
        
    def draw_torque_diagram(self):
        
        self.canvas.axes.cla()  #clear canvas
        self.canvas.axes.set_title("Torque diagram")
        self.canvas.axes.set_ylabel(r"Torque T / Nm")
        self.canvas.axes.set_xlabel(r"Time / $s$")
        self.canvas.axes.plot(self.time, self.total_torque, color ='red')
        self.canvas.axes.grid(True, which='both', linestyle='--', linewidth=0.5)
        self.canvas.axes.minorticks_on()
        self.canvas.draw()
        
    def draw_shear_rate_diagram(self):
        
        
        self.canvas.axes.cla()  #clear canvas
        self.canvas.axes.set_title("Shear rate diagram")
        self.canvas.axes.set_ylabel(r"Shear rate $\dot\gamma$ / $s^{-1}$")
        self.canvas.axes.set_xlabel(r"Time / $s$")
        self.canvas.axes.plot(self.time,self.shear_rate, color ='red')
        self.canvas.axes.grid(True, which='both', linestyle='--', linewidth=0.5)
        self.canvas.axes.minorticks_on()
        self.canvas.draw()


    def draw_shear_stress_diagram(self):
        self.canvas.axes.cla()  # clear canvas
        self.canvas.axes.set_title("Shear stress diagram")
        self.canvas.axes.set_ylabel(r"Shear stress $\tau$ / Pa")
        self.canvas.axes.set_xlabel(r"Time / $s$")
        self.canvas.axes.plot(self.time, self.shear_stress , color='red')
        self.canvas.axes.grid(True, which='both', linestyle='--', linewidth=0.5)
        self.canvas.axes.minorticks_on()
        self.canvas.draw()

    def draw_viscosity_diagram(self):
        self.canvas.axes.cla()  # clear canvas
        self.canvas.axes.set_title("Viscosity diagram")
        self.canvas.axes.set_ylabel(r"Viscosity $\eta$")
        self.canvas.axes.set_xlabel(r"Time / $s$")
        self.canvas.axes.plot(self.time, self.viscosity, color='red')
        self.canvas.axes.grid(True, which='both', linestyle='--', linewidth=0.5)
        self.canvas.axes.minorticks_on()
        self.canvas.draw()


    def closeEvent(self, event):
        event.accept()
        

def main_3():
    app3 = QApplication([])
    
    # get absolute path of project root (folder containing 'src' and 'pics')
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    # construct icon path
    fzj_icon_path = os.path.join(project_root, "pics", "fzj.ico")
    app3.setWindowIcon(QtGui.QIcon(fzj_icon_path))
    window3 = AnalyseWindow()
    window3.show()
    app3.exec_()

if __name__ == '__main__':
    main_3()