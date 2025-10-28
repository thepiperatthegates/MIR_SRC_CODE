"""
Main entry point for GUI_SRC_CODE.

Responsibilities:
#. Manage GUI-related classes and functions.
#. Handle live data plotting.
#. Provide USB transmission functionality.

"""
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
from gui_baru import Ui_Title
from main_window_test import Ui_MainWindow

import sockets_files 
from sockets_files import q_to_graph

import packet_transmission as packet_transmission
from window_show import AnalyseWindow
from graph_popout import PlotWindow
from creep_test_window import CreepTestGUI



# GLOBAL VARIABLES
checkbox_variable = 0
time_receive_thread = 0


data_1 = 0
data_2 = 0
data_3 = 0
data_4 = 0       #offset 1
data_5 = 0
data_6 = 0      #offset 2
data_7 = 0
data_8 = 0
data_9 = 0
data_10 = 0    #mir mode

time_slice = np.array([], dtype=np.float32)
v1_slice   = np.array([], dtype=np.uint16)
v2_slice   = np.array([], dtype=np.uint16)
i1_slice   = np.array([],  dtype=np.uint16)
i2_slice   = np.array([], dtype=np.uint16)

store_array1 = np.array([], dtype=np.uint16)
store_array2 = np.array([], dtype=np.uint16)
store_array3 = np.array([], dtype=np.uint16)
store_array4 = np.array([], dtype=np.uint16)



angle_permanent_magnet_val = np.array([], dtype=np.float32)
angle_magnetic_field_val = np.array([], dtype=np.float32)
phase_difference_val = np.array([], dtype=np.float32)

time_axis =  [i * 0.0001 for i in range(1000)]

bytes_to_process = np.array([], dtype=np.uint16)  # Empty NumPy array for incoming data

flag_done = None

data_mutex = QMutex()


#function receiving data through pipe from another thread
class DataUpdate(QThread):
    def __init__(self, main_window_ref):
        super().__init__()
        self.main_window = main_window_ref
        self.running = True
        self.flag_calibrate = False


        self.flag_fR_measurement = False
        self.accumulate_hall_1 = None
        self.accumulate_hall_2 = None
        self.accumulate_current_1 = None
        self.accumulate_current_2 = None
        
        self.flag_normalise = False
        

        self.total_hall_1 = None
        self.total_hall_2 = None
        self.total_current_1 = None
        self.total_current_2 = None

        


    def run(self):
        global bytes_to_process, time_slice, v1_slice, v2_slice, i1_slice, i2_slice
        global store_array1, store_array2, store_array3, store_array4
        global angle_permanent_magnet_val, angle_magnetic_field_val, phase_difference_val
        global data_3, data_5
        

        
        num_columns=4

        data_from_pipe = [] #creating a list here because data from pipe is a list
        while self.running:
            data_from_pipe = q_to_graph.get()
            if not self.running:
                break
            if data_from_pipe:
                bytes_to_process = data_from_pipe   #now changes to np array so we can work with it better

                trimmed_size = len(bytes_to_process) - (len(bytes_to_process) % num_columns)
                bytes_to_process = bytes_to_process[:trimmed_size]

                if len(bytes_to_process) == 0:
                    return
                reshaped_data = np.array(bytes_to_process).reshape(-1, num_columns)
                reshaped_data = reshaped_data.astype(float)

                v1_slice = reshaped_data[:, 0]
                v2_slice= reshaped_data[:, 1]
                i1_slice= reshaped_data[:, 2] #STIMMT
                i2_slice= reshaped_data[:, 3]   #STIMMT
    
                data_mutex.lock()

                #Hall Sensors
                store_array1= -packet_transmission.change_adc_hall(v1_slice)               #convert col1 (in V)
                store_array2 = packet_transmission.change_adc_hall( v2_slice)               #convert col2 (in V)
                
                #Current
                store_array3 =  -packet_transmission.change_current_adc(i1_slice)               #convert col3 (in mA)
                store_array4  = packet_transmission.change_current_adc(i2_slice)               #convert col4 (in mA)
                
                
                #calibration for current sensor 
                store_array3 = packet_transmission.calibration_input_coil_1(store_array3)
                store_array4 = packet_transmission.calibration_input_coil_2(store_array4)
                

                #Calibrate process starts
                if self.flag_calibrate:
                    self.calculate_calibration(store_array1, store_array2, store_array3, store_array4)
                
                #measurement fR process starts
                if self.flag_fR_measurement:
                    self.calculate_fR(store_array1, store_array2, store_array3, store_array4)
            
                #Calibrated hall sensors
                store_array1 = packet_transmission.calibrated_hall_sensors1(store_array1, store_array3/1000)  
                store_array2 = packet_transmission.calibrated_hall_sensors2(store_array2, store_array4/1000)
                
                
                #this is normalising step (still do not know whether i want to do it immidiately or not)
                if self.flag_normalise == True:
                    amplitude_voltage_1 = (np.max(a=store_array1) - np.min(store_array1)) / 2
                    zero_offset_voltage_1 = (np.max(store_array1) + np.min(store_array1)) / 2

                    amplitude_voltage_2 = (np.max(store_array2) - np.min(store_array2)) / 2
                    zero_offset_voltage_2 = (np.max(store_array2) + np.min(store_array2)) / 2

                    store_array1 = (store_array1 - zero_offset_voltage_1) / amplitude_voltage_1
                    store_array2 = (store_array2 - zero_offset_voltage_2) / amplitude_voltage_2
                    
                #######################################################################################################
                angle_permanent_magnet_val = np.arctan2(store_array2, store_array1)
                angle_magnetic_field_val = np.arctan2(store_array4, store_array3)
                
                angle_permanent_magnet_val = np.unwrap(angle_permanent_magnet_val)
                angle_magnetic_field_val  = np.unwrap(angle_magnetic_field_val)
                #######################################################################################################
                phase_difference_val = angle_magnetic_field_val - angle_permanent_magnet_val
                data_mutex.unlock()
                
                #Clear queue after processing
                bytes_to_process = np.array([], dtype=np.uint16)


    def calculate_calibration(self, store_array1_calibrate, store_array2_calibrate, store_array3_calibrate, store_array4_calibrate):

        self.accumulate_hall_1 = np.append(self.total_hall_1, store_array1_calibrate)
        self.accumulate_hall_2 = np.append(self.total_hall_2, store_array2_calibrate)
        self.accumulate_current_1 = np.append(self.total_current_1, store_array3_calibrate)
        self.accumulate_current_2 = np.append(self.total_current_2, store_array4_calibrate)
        
        self.main_window.set_constant(self.accumulate_hall_1,  
                                    self.accumulate_hall_2,
                                    self.accumulate_current_1,
                                    self.accumulate_current_2)
        
    def calculate_fR(self, store_array1_calibrate, store_array2_calibrate, store_array3_calibrate, store_array4_calibrate):
        self.accumulate_hall_1 = np.append(self.total_hall_1, store_array1_calibrate)
        self.accumulate_hall_2 = np.append(self.total_hall_2, store_array2_calibrate)
        self.accumulate_current_1 = np.append(self.total_current_1, store_array3_calibrate)
        self.accumulate_current_2 = np.append(self.total_current_2, store_array4_calibrate)
        
        self.main_window.set_constant(self.accumulate_hall_1,  
                                    self.accumulate_hall_2,
                                    self.accumulate_current_1,
                                    self.accumulate_current_2)
        
    def flag_special_event(self, flag_1, flag_2):
        
        #Set flag for calibration
        self.flag_calibrate = flag_1
        #Set flag for fR measurement
        self.flag_fR_measurement = flag_2

    def flag_normalise_event(self, flag_input):
        
        self.flag_normalise = flag_input

    def stop(self):
        self.running = False

        
class SleepTimer(QObject):
    update_time_signal = pyqtSignal(float)  # emit float countdown values

    def __init__(self, parent=None):
        super().__init__(parent)
        global data_1
        self.remaining = float(data_1) # get local_data_1 from global
        self.timer = QTimer(self)
        self.timer.setInterval(100)  # 100 ms per tick
        self.timer.timeout.connect(self._tick)

<<<<<<< HEAD
    def run(self):
        #TODO: implement a way to kill the thread when stop button is pressed !
        global data_1     #data for run time
        # packet_transmission.running_time_event(1)
        local_data_1 = int(data_1)-1       #data for run time
        while local_data_1 >=0:
            if self.running:
                time.sleep(1)
                self.update_time_signal.emit(int(local_data_1))
                local_data_1 -= 1
            else:
                break
        packet_transmission.running_time_event(0)
=======
    def start(self):
        self.timer.start()
>>>>>>> dev

    def stop(self):
        self.timer.stop()

    def _tick(self):
        self.remaining -= 0.1
        if self.remaining >= 0.0:
            self.update_time_signal.emit(round(self.remaining, 1))
        else:
            self.timer.stop()
            packet_transmission.running_time_event(0)

class SocketThread(QThread):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.running = True

    def run(self):
        if self.running:
            sockets_files.thread_start()


    def stop(self):
        self.running = False
    


class ConstShearGUI(QMainWindow, Ui_Title):

    queue_file_name = multiprocessing.Queue()

    def __init__(self):
        super().__init__()
        self.setupUi(self)
        
        if hasattr(QtCore.Qt, 'AA_EnableHighDpiScaling'):
            QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
        if hasattr(QtCore.Qt, 'AA_UseHighDpiPixmaps'):
            QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)

        # get absolute path of project root (folder containing 'src' and 'pics')
        self.project_root = os.path.dirname(os.path.abspath(__file__))

        # construct icon path
        save_icon_path = os.path.join(self.project_root, "pics", "save_icon.ico")
        calib_icon_path = os.path.join(self.project_root, "pics", "calibrate.png")
        fr_icon_path = os.path.join(self.project_root, "pics", "friction_icon.png")
        
        self.save_button.setIcon(QtGui.QIcon(save_icon_path))
        self.button_fr_constant.setIcon(QtGui.QIcon(fr_icon_path))
        self.button_cal_constant.setIcon(QtGui.QIcon(calib_icon_path))

        self.setWindowTitle("Const Shear Rate Window")
        
        self.button_stop.setDisabled(True)

        ###################################declare processes here first to keep a reference to the process
        self.p_window_data = None
        self.p_analyse = None
        self.worker_sleep = None
        self._plot_ref1 = None       
        self._plot_ref2 = None
        self.before1 = None
        self.before2 = None
        self.curve_v1 = self.curve_v2 = self.curve_i1 = self.curve_i2 = self.curve_sigma_b = self.curve_sigma_m = None
        
        self.flag_fR = False
        self.flag_K = False
        
        ####### variables for reference class DataUpdate(QThread) ###########################################        
        self.accumulate_hall_1 = None       #in V
        self.accumulate_hall_2  = None      #in V
        self.accumulate_current_1 = None    #in mA
        self.accumulate_current_2 = None    #in mA
        
        ####### variables for calibration process ###########################################
    
        self.mean_hall_1_0_A = None         #in V     
        self.mean_hall_2_0_A = None         #in V
        self.mean_hall_1_400_A = None       #in mA
        self.mean_hall_2_0_A = None         #in mA
        
        
        ####### variables for fR measurement process ###############################################
        self.mean_current1_fR = None        #in mA
        self.mean_current2_fR = None        #in mA
        self.mean_hall1_fR = None           #in V
        self.mean_hall2_fR = None           #in V 
        self.calculated_torque = np.zeros(20)               #in 
        self.calculated_angular_velocity = np.zeros(20)
        self.mean_phase = np.zeros(20)
        self.standard_mean_phase = np.zeros(20)
        self.standard_mean_torque = np.zeros(20)
        self.standard_torque = np.zeros(20)
        self.standard_phase = np.zeros(20)
        self.calculate_final_fR = 0
        
        
        self.COIL_CONSTANT = packet_transmission.COIL_CONSTANT		# in T / A
        self.DIPOLE_MOMENT = packet_transmission.DIPOLE_MOMENT	# in A m^2
        self.CALIBRATION_FACTOR = packet_transmission.CALIBRATION_FACTOR	# torque calibration no units (K)
        
        

        #########################################################################################
        #placeholder text for textboxes################################################
        self.textbox_time.setPlaceholderText("Enter time in second")
        self.textbox_frequency.setPlaceholderText("Enter frequency for coil currents")
        self.textbox_amplitude1.setPlaceholderText("Enter amplitude from 0 to 480mA")
        self.textbox_offset1.setPlaceholderText("Enter offset +-480mA")
        self.textbox_amplitude2.setPlaceholderText("Enter amplitude from 0 to 480mA")
        self.textbox_offset2.setPlaceholderText("Enter offset +-480mA")
        self.k_b_label.setText(
            f"k<sub>b1</sub> = {packet_transmission.k_b_1}&nbsp;&nbsp;&nbsp;"
            f"k<sub>b2</sub> = {packet_transmission.k_b_2}&nbsp;&nbsp;&nbsp;"
            f"K = {packet_transmission.CALIBRATION_FACTOR}&nbsp;&nbsp;&nbsp;"
            f"f<sub>R</sub> equation = {np.poly1d(self.calculate_final_fR)}"
        )
        self.label_frequency.setText("Shear rate γ̇ / s<sup>-1</sup>")    
        self.button_fr_constant.setText("Measure f_R")
        self.textbox_sample_frequency.setText("10000")
        #################################################################################################

        #######################################################validator############################################################################
        validator = QDoubleValidator(0.0, 1000.0, 1)  # min=0.0, max=1000.0, 2 decimals
        validator.setNotation(QDoubleValidator.StandardNotation)

        self.textbox_time.setValidator(validator)
        self.textbox_frequency.setValidator(QDoubleValidator())
        
        #ACCEPT ONLY FLOAT FROM 0 TO 480 (unsigned)
        self.input_validator_unsigned = QRegularExpressionValidator(    QRegularExpression(r"^(?:[0-9](?:\.[0-9]+)?|[1-9][0-9](?:\.[0-9]+)?|[1-3][0-9]{2}(?:\.[0-9]+)?|4[0-7][0-9](?:\.[0-9]+)?|480(?:\.0+)?)$"), self)
        #ACCEPT ONLY INTEGER FROM -480 to 480 (signed)
        self.input_validator_signed = QRegularExpressionValidator(QRegularExpression(r"^-?(?:[0-9]|[1-9][0-9]|[1-3][0-9]{2}|4[0-7][0-9]|480)$"), self)
        self.textbox_amplitude1.setValidator(self.input_validator_unsigned)
        self.textbox_offset1.setValidator(self.input_validator_signed)
        self.textbox_amplitude2.setValidator(self.input_validator_unsigned)
        self.textbox_offset2.setValidator(self.input_validator_signed)
        #######################################################validator############################################################################
        
        
        #### connected functions for button
        self.button_send.clicked.connect(self.send_parameter_event)
        self.button_send.clicked.connect(lambda value: self.popout_window(1))
        
        self.button_start.clicked.connect(self.start_data_event)
        self.button_start.clicked.connect(lambda value: self.popout_window(1))
        
        self.button_stop.clicked.connect(self.stop_button_push_event)
        self.button_stop.clicked.connect(lambda  value: self.popout_window(1))
        
<<<<<<< HEAD
<<<<<<< Updated upstream
        #calibration button
        self.button_cal_constant.clicked.connect(self.start_calibration_event)
=======
        ### normalise button
        self.normalise_button.clicked.connect(self.start_normalise_event)
        
        ### calibration button
        self.button_cal_constant.clicked.connect(lambda value: self.start_calibration_event(input_current=-400, count_recursion = 0))
        self.button_cal_constant.clicked.connect(lambda  value: self.popout_window(1))

        ### friction coefficient rechnung gedrückt
        self.button_fr_constant.clicked.connect(lambda value: self.start_friction_coeff_event (1, 1, 0, 30))
        self.button_fr_constant.clicked.connect(lambda  value: self.popout_window(arg=3))
        
        
        
        ### clicked to open analyse data window
>>>>>>> Stashed changes
        self.analyse_button.clicked.connect(self.analyse_button_event)
        
        #normalise button
=======
        ### normalise button
>>>>>>> dev
        self.normalise_button.clicked.connect(self.start_normalise_event)
        
        ### calibration button
        self.button_cal_constant.clicked.connect(lambda value: self.start_calibration_event(input_current=-400, count_recursion = 1))
        self.button_cal_constant.clicked.connect(lambda  value: self.popout_window(1))

        ### friction coefficient rechnung gedrückt
        self.button_fr_constant.clicked.connect(lambda value: self.start_friction_coeff_event_initiation (running_frequency=1, rotation_direction=1, count_recursion=0))
        self.button_fr_constant.clicked.connect(lambda  value: self.popout_window(arg=3))
        
        self.stop_default_state = 1
        self.graph_stop_button.clicked.connect(self.graph_stop_event)
        
        self.actionHardware_reset.triggered.connect(self.set_hardware_reset_event)
        self.actionSoftware_restart.triggered.connect(self.set_software_reset_event)
        
        self.button_auto_range.clicked.connect(self.auto_range_event)
        
        self.save_button.clicked.connect(self.save_button_event)

        #Start backend serial lines
        
        self.worker_socket = SocketThread()
        self.worker_DataUpdate = DataUpdate(self)
        
        self.worker_socket.start()
        self.worker_DataUpdate.start()
        
        
        #combobox for live graph mode
        self.select_mode_comboBox.activated.connect(self.change_graph)
        #same as above but for time interval change
        self.timeInterval_comboBox.activated.connect(self.change_graph)
        
    def change_graph(self):
        global time_axis
        
        #get string from combo box
        mode = self.select_mode_comboBox.currentText()
        time_interval_var_string = self.timeInterval_comboBox.currentText()
        time_interval_var_int = int(time_interval_var_string[:-2])
        
        
        #clear graph and stop every time the event is connected
        self.graphicsView.clear()
        if hasattr(self, 'timer'):
            self.timer.stop()

    
        if mode == "View sensors":
            
            #this is to make sure x-axis is configured properly 
            if time_interval_var_int == 100:
                sockets_files.tot_count_accumulate_recv = 250
                time_axis = [i * 0.0001 for i in range(1000)]
            elif time_interval_var_int == 500:
                sockets_files.tot_count_accumulate_recv = 5*250
                time_axis = [i * 0.0001 for i in range(5*1000)]
            elif time_interval_var_int == 1000:
                sockets_files.tot_count_accumulate_recv = 10*250
                time_axis = [i * 0.0001 for i in range(10*1000)]
            
            ######################
            self.plot1= self.graphicsView.addPlot(row=0, col=0, title="Hall sensors")
            self.plot1.setLabel('left', 'Voltage', units='V')
            self.plot1.setLabel('bottom', 'Time', units= 's')
            self.plot1.addLegend()
            self.plot1.showGrid(x=True, y=True)
            self.curve_v1 = self.plot1.plot(pen='r', name="Hall sensors 1")
            self.curve_v2 = self.plot1.plot(pen='b', name="Hall sensors 2")

            
            self.plot2 = self.graphicsView.addPlot(row=1, col=0, title="Current sensors")
            self.plot2.setLabel('left', 'Current', units='mA')
            self.plot2.setLabel('bottom', 'Time', units= 's')
            self.plot2.addLegend()
            self.plot2.showGrid(x=True, y=True)
            
            
            self.curve_i1 = self.plot2.plot(pen='g', name="I1")
            self.curve_i2 = self.plot2.plot(pen='y', name="I2")
            
            self.plot1.enableAutoRange(axis='x', enable=False)
            self.plot2.enableAutoRange(axis='x', enable=False)
            self.plot1.setXRange(0, 0.5)
            self.plot2.setXRange(0, 0.5)
        
            # Create a timer for sensor updates
            self.timer = QtCore.QTimer()
            self.timer.setInterval(time_interval_var_int)  # 0.5 seconds interval
            self.timer.timeout.connect(self.graph_update_sensors)
            self.timer.start()

        elif mode == "View angle":
            
            #this is to make sure x-axis is configured properly 
            if time_interval_var_int == 100:
                sockets_files.tot_count_accumulate_recv = 250
                time_axis = [i * 0.0001 for i in range(1000)]
            elif time_interval_var_int == 500:
                sockets_files.tot_count_accumulate_recv = 5*250
                time_axis = [i * 0.0001 for i in range(5*1000)]
            elif time_interval_var_int == 1000:
                sockets_files.tot_count_accumulate_recv = 10*250
                time_axis = [i * 0.0001 for i in range(10*5000)]
                
            self.plot1= self.graphicsView.addPlot(row=0, col=0, title="Permanent magnet angle")
            self.plot1.setLabel('left', 'ϕ_m', units='rad')
            self.plot1.setLabel('bottom', 'Time', units= 's')
            self.plot1.addLegend()
            self.plot1.showGrid(x=True, y=True)
            self.curve_sigma_m = self.plot1.plot(pen='r', name="Sigma")
    
            
            self.plot2 = self.graphicsView.addPlot(row=1, col=0, title="Magnetic field angle")
            self.plot2.setLabel('left', 'ϕ_B', units='rad')
            self.plot2.setLabel('bottom', 'Time', units= 's')
            self.plot2.addLegend()
            self.plot2.showGrid(x=True, y=True)
            self.curve_sigma_b = self.plot2.plot(pen='g', name="B")
            
            self.plot1.enableAutoRange(axis='x', enable=False)
            self.plot2.enableAutoRange(axis='x', enable=False)
            self.plot1.setXRange(0, 0.5)
            self.plot2.setXRange(0, 0.5)
            
            self.timer = QtCore.QTimer()
            self.timer.setInterval(time_interval_var_int)  # 0.5 seconds interval
            self.timer.timeout.connect(self.graph_update_angle)
            self.timer.start()


            
        elif mode == "View phase difference":
            
            
            #this is to make sure x-axis is configured properly 
            #this is to make sure x-axis is configured properly 
            if time_interval_var_int == 100:
                sockets_files.tot_count_accumulate_recv = 250
                time_axis = [i * 0.0001 for i in range(1000)]
            elif time_interval_var_int == 500:
                sockets_files.tot_count_accumulate_recv = 5*250
                time_axis = [i * 0.0001 for i in range(5*1000)]
            elif time_interval_var_int == 1000:
                sockets_files.tot_count_accumulate_recv = 10*250
                time_axis = [i * 0.0001 for i in range(10*1000)]
                
            self.plot1= self.graphicsView.addPlot(row=0, col=0, title="Permanent magnet angle")
            self.plot1.setLabel('left', 'Δϕ', units='rad')
            self.plot1.setLabel('bottom', 'Time', units= 's')
            self.plot1.showGrid(x=True, y=True)
            self.curve_phase_difference = self.plot1.plot(pen='g')
            self.plot1.addLegend()
        
            self.plot1.enableAutoRange(axis='x', enable=False)
            self.plot1.setXRange(0, 0.5)

            # Create a timer for angle updates
            self.timer = QtCore.QTimer()
            self.timer.setInterval(time_interval_var_int)  # 0.5 seconds interval
            self.timer.timeout.connect(self.graph_phase_difference)
            self.timer.start()
            

    def graph_update_sensors(self):
        
        global store_array1, store_array2, store_array3, store_array4, time_axis


        self.curve_v1.setData(time_axis,store_array1)
        self.curve_v2.setData(time_axis,store_array2)
        
        self.curve_i1.setData(time_axis,store_array3)
        self.curve_i2.setData(time_axis,store_array4)
        
    def auto_range_event(self):
        
        self.plot1.enableAutoRange(axis='y', enable=True)
        self.plot2.enableAutoRange(axis='y', enable=True)
        # self.plot1.enableAutoRange(axis='x', enable=True)
        # self.plot2.enableAutoRange(axis='x', enable=True)
        

    def graph_update_angle(self):
        global angle_permanent_magnet_val, angle_magnetic_field_val
        
        self.curve_sigma_m.setData(time_axis, angle_permanent_magnet_val)
        self.curve_sigma_b.setData(time_axis, angle_magnetic_field_val)
        
    def graph_phase_difference(self):
        global phase_difference_val
        
        self.curve_phase_difference.setData(time_axis, phase_difference_val)
        
        
    def send_parameter_event(self):
        
        """ 
        Send data event to STM32H7575XI MCU/backend such as the amplitude and offsets for the two pair of coils, the frequency of DAC, the 
        rotation of the magnet and whether to use FIR filtering or not. 
        
        """
        global data_1
        global data_2
        global data_3
        global data_4
        global data_5
        global data_6
        global data_7        #checkbox for direction
        global data_8
        global data_9
        global data_10
    
        data_1 = 65534
        data_2 =self.textbox_frequency.text()
        data_2 = packet_transmission.calculate_running_frequency(float(data_2))
        data_3 = self.textbox_amplitude1.text()
        data_4 =  self.textbox_offset1.text()
        data_5 = self.textbox_amplitude2.text()
        data_6 = self.textbox_offset2.text()  

        
        #from combobox direction
        if self.comboBox_direction.currentText() == "Clockwise":
            data_7 = 2
        elif self.comboBox_direction.currentText() == "Anti-clockwise":
            data_7 = 1
            
        if self.filter_checkbox.isChecked():
            data_8 = 0
        else:
            data_8 = 2
            
<<<<<<< HEAD
        data_10 =1
=======
        data_10 = 1
        
        ##enabled stop rotating button 
        self.button_stop.setDisabled(False)
>>>>>>> dev
        
        
        packet_transmission.send_function(data_1, data_2, data_3, data_4, data_5, data_6, data_7, data_8, data_9, data_10)
        #send all the data to be packed
        packet_transmission.send_transmission_event(1)            #SET flag for Tx
        packet_transmission.start_flag_send_event(1)
        

        self.status_label.setStyleSheet("color: #32a83a;")
        self.status_label.setText("Data sent!")
    
    def start_data_event(self):
        
        
        #### delete previous dummy files before acquistion starts. 
        path_join =  os.path.join(self.project_root, "files", "dummy.csv")
        #remove the dummy if it exists
        if os.path.exists(path_join):
            try:
                os.remove(path_join)
                print("dummy.csv deleted")
            except OSError as e:
                print(f"Error deleting file csv: {e}")
        else:
            print("First measurements")
            
            
        global data_1
        global data_2
        global data_3
        global data_4
        global data_5
        global data_6
        global data_7
        global data_8
        global data_9
        global data_10
        
        data_1 = self.textbox_time.text()
        data_2 =self.textbox_frequency.text()
        data_2 = packet_transmission.calculate_running_frequency(float(data_2))
        data_3 = self.textbox_amplitude1.text()
        data_4 =  self.textbox_offset1.text()
        data_5 = self.textbox_amplitude2.text()
        data_6 = self.textbox_offset2.text() 
        
<<<<<<< HEAD
        data_10 = 10
        
        packet_transmission.send_function(data_1, data_2, data_3, data_4, data_5, data_6, data_7, data_8, data_9, data_10)
    
        packet_transmission.stop_button_event(0)            #goto sockets_files and stop the loop for receiving
        packet_transmission.running_time_event(1)
        
        
        ### get the desired sampling frequency
        
        sampling_frequency = float(self.textbox_sample_frequency.text())
        sockets_files.time_increment = 1.0/sampling_frequency
        sockets_files.tot_average = int(10000.0/ sampling_frequency)
        print("tot_average", sockets_files.tot_average)
        
        sockets_files.file_name_change_set("dummy")        #set file name from gui
        sockets_files.current_time = 0.0
=======
        data_10 = 1
        packet_transmission.send_function(data_1, data_2, data_3, data_4, data_5, data_6, data_7, data_8, data_9, data_10)
>>>>>>> dev

        ######## get the desired sampling frequency from the textbox
        sampling_frequency = self.textbox_sample_frequency.text()
        if sampling_frequency == '' or sampling_frequency == 0:
            pass
        else:
                
            sockets_files.time_increment = 1.0/float(sampling_frequency)
            sockets_files.tot_average = 10000 // int(sampling_frequency)
            print("tOT_average", sockets_files.tot_average)
            
            check = 5000 / int(sockets_files.tot_average)
            print("check is", check)
        
        if check.is_integer():
            packet_transmission.stop_button_event(0)            #goto sockets_files and stop the loop for receiving
            packet_transmission.running_time_event(this_running_time_flag=1)
            
            sockets_files.file_name_change_set("dummy")        #set file name from gui
            sockets_files.current_time = 0.0

            self.status_label.setStyleSheet("color: #7da832;")
            self.status_label.setText("Acquisition starts.......")


<<<<<<< HEAD
        self.queue_file_name.put("dummy") #Send file name to another process
        
<<<<<<< Updated upstream
        
        
        
=======
            self.queue_file_name.put("dummy") #Send file name to another process
            
            #start the process at the initialisation
            #FOR NOW, LETS NOT DO ACQUISTION WINDOW
            self.worker_sleep = SleepTimer()
            self.worker_sleep.update_time_signal.connect(self.update_time_counter_acquisition)
            self.worker_sleep.start()
>>>>>>> dev
        
        ## Error handling when invalid sampling frequency is being input
        else:
            self.popout_window(5)
        
    def update_time_counter_acquisition(self, val):
        self.lcdNumber.display(val)
            
        if val in (0, 0.1):
            self.button_start.setDisabled(False)
            self.button_stop.setDisabled(False)
        else:
            self.button_start.setDisabled(True)
            self.button_stop.setDisabled(True)

<<<<<<< HEAD
        #start the process at the initialisation
=======
>>>>>>> Stashed changes
        #FOR NOW, LETS NOT DO ACQUISTION WINDOW
=======
    def start_normalise_event(self):
        if self.worker_DataUpdate.flag_normalise == False:
            self.worker_DataUpdate.flag_normalise_event(True)
        else:
            self.worker_DataUpdate.flag_normalise_event(False)
>>>>>>> dev

    def popout_window(self, arg):
        msg = QMessageBox()
        if arg == 1:
            msg.setText("Successful")
        elif arg == 2:
            msg.setText(f"k b 1 = {packet_transmission.k_b_1}\n"
                f"k b 2 = {packet_transmission.k_b_2}")
        elif arg == 3:
            msg.setText(f"Friction coefficient measurement is starting innit")
        elif arg == 4:
            msg.setText(f"f_R: {np.poly1d(self.calculate_final_fR)}")
        elif arg == 5:
            msg.setText("Invalid sample frequency! Please do not use any value starting with 3, 7 or 9!")
            
        msg.setIcon(QMessageBox.Question)
        
        x = msg.exec_()

    def start_calibration_event(self, input_current = -400, count_recursion = 1 ):
        global data_1
        global data_2
        global data_3
        global data_4
        global data_5
        global data_6
        global data_7
        global data_8
        global data_9
        global data_10
        print("Recursion in main func:", count_recursion)
        
        packet_transmission.k_b_1 = 0
        packet_transmission.k_b_2 = 0
        
        if input_current == False:
            input_current = -400
            
        
        print("Input current for calibration:", input_current)
        
        data_1 = str(1) #seconds
        data_2 = str(3) # Hz
        data_3 = str(input_current)# mA
        data_4 = str(0)
        data_5 = str(input_current)  # mA
        data_6 = str(0)  # mA 

        #from combobox direction
        if self.comboBox_direction.currentText() == "Clockwise":
            data_7 = 2
        elif self.comboBox_direction.currentText() == "Anti-clockwise":
            data_7 = 1
            
        data_8 = 3          #mode 3 to the board (for dc generator)
        data_9= 0
<<<<<<< HEAD
        data_10 = q_get_mir_mode.get()
=======
        data_10 = 1
>>>>>>> dev

        
        packet_transmission.send_function(data_1, data_2, data_3, data_4, data_5, data_6, data_7, data_8, data_9, data_10)
        # send all the data to be packed
        packet_transmission.send_transmission_event(this_flag_send=1)            #SET flag for Tx
        packet_transmission.start_flag_send_event(1)

        # send flag for calibration in the thread
        self.worker_DataUpdate.flag_special_event(flag_1=True, flag_2=False)

        self.status_label.setStyleSheet("color: #32a83a;")
        self.status_label.setText("Calibrating!...............")
        
<<<<<<< HEAD
<<<<<<< Updated upstream
        QtCore.QTimer.singleShot(1000, lambda: self.after_stabilise(count_recursion))
=======
        QtCore.QTimer.singleShot(10000, lambda: self.after_stabilise_calibration(count_recursion))
>>>>>>> Stashed changes
=======
        QtCore.QTimer.singleShot(2000, lambda: self.after_stabilise_calibration(count_recursion))
>>>>>>> dev
        

    def after_stabilise_calibration(self, count_recursion):
        self.worker_sleep = SleepTimer()
        self.worker_sleep.update_time_signal.connect(lambda value: self.update_time_counter_calibrating(value, count_recursion))
        self.worker_sleep.start()
        

    def update_time_counter_calibrating(self, val, count_recursion):
        self.lcdNumber.display(val)
        print("Recursion in called func:", count_recursion)

        if val != 0.0:
            self.button_send.setDisabled(True)
            self.button_start.setDisabled(True)
            self.button_stop.setDisabled(False)

        elif val == 0.0:  #when val is 0 and the thread is stops already
            
            #reset flags
            self.worker_DataUpdate.flag_special_event(False, False)

            if count_recursion == 1:
                self.mean_hall_1_0_A = np.mean(self.accumulate_hall_1[1000:])
                self.mean_hall_2_0_A = np.mean(self.accumulate_hall_2[1000:])
                
                self.mean_current_1_0_A = np.mean(self.accumulate_current_1[1000:])
                self.mean_current_2_0_A = np.mean(self.accumulate_current_2[1000:])
    
                ### second recursion occurs
                self.start_calibration_event(400, 2)
            elif count_recursion == 2:
                self.mean_hall_1_400_A = np.mean(self.accumulate_hall_1[1000:])
                self.mean_hall_2_400_A = np.mean(self.accumulate_hall_2[1000:])
                
                
                self.mean_current_1_400_A = np.mean(self.accumulate_current_1[1000:])
                self.mean_current_2_400_A = np.mean(self.accumulate_current_2[1000:])
                
                
                k_b_1 = float ((self.mean_hall_1_400_A - self.mean_hall_1_0_A) / ((self.mean_current_1_400_A - self.mean_current_1_0_A)/1000))
                k_b_2 = float ((self.mean_hall_2_400_A - self.mean_hall_2_0_A) / ((self.mean_current_2_400_A - self.mean_current_2_0_A)/1000))
                
                packet_transmission.k_b_1 = k_b_1
                packet_transmission.k_b_2 = k_b_2
            
                
                print(k_b_1)
                print(k_b_2)
                
                self.k_b_label.setText(
                    f"k<sub>b1</sub> = {packet_transmission.k_b_1}&nbsp;&nbsp;&nbsp;"
                    f"k<sub>b2</sub> = {packet_transmission.k_b_2}&nbsp;&nbsp;&nbsp;"
                    f"K = {packet_transmission.CALIBRATION_FACTOR}&nbsp;&nbsp;&nbsp;"
                    f"f<sub>R</sub> equation = {np.poly1d(self.calculate_final_fR)}"
                )
                
<<<<<<< HEAD
<<<<<<< Updated upstream
                self.popout_window_calibration()
=======
=======
>>>>>>> dev
                self.popout_window(2)
                #enable the button again
                self.button_send.setDisabled(False)
                self.button_start.setDisabled(False)
<<<<<<< HEAD
                self.button_stop.setDisabled(True)
=======
                self.button_stop.setDisabled(False)
>>>>>>> dev
                    
    def set_constant(self, get_accumulate_hall_1, get_accumulate_hall_2, get_accumulate_current_1, get_accumulate_current2):
        """
        Set accumulated measurement values for Hall sensors and current sensors.

        This method updates the internal state variables that store accumulated
        readings for two Hall sensors and two current sensors. It is shared by
        both ``update_time_counter_fR_measurement`` and
        ``update_time_counter_calibrating``.
        
        Shared by function update_time_counter_fR_measurement and update_time_counter_calibrating

        :param float get_accumulate_hall_1: Accumulated value from Hall sensor 1.
        :param float get_accumulate_hall_2: Accumulated value from Hall sensor 2.
        :param float get_accumulate_current_1: Accumulated value from current sensor 1.
        :param float get_accumulate_current2: Accumulated value from current sensor 2.
        """
        
        
        self.accumulate_hall_1 = get_accumulate_hall_1
        self.accumulate_hall_2 = get_accumulate_hall_2
        self.accumulate_current_1 = get_accumulate_current_1
        self.accumulate_current_2 = get_accumulate_current2
<<<<<<< HEAD

    def start_friction_coeff_event(self, running_frequency = 1,  rotation_direction =  1, count_recursion = 0, input_current = 30):
=======
    
    
    #initiation!!!!!
    def start_friction_coeff_event_initiation(self, running_frequency = 1,  rotation_direction =  1, count_recursion = 0, input_current = 30):
>>>>>>> dev
        global data_1
        global data_2
        global data_3
        global data_4
        global data_5
        global data_6
        global data_7
        global data_8
        global data_9
        global data_10
        
<<<<<<< HEAD
        packet_transmission.f_R = 0
        
        ###calculate running time for 10 rotation
        
        data_1 = str(self.running_time_10_rotation(running_frequency))
        print(data_1)
        data_2 = str(running_frequency) # Hz
        data_3 = str(input_current)# mA
        data_4 = self.textbox_offset2.text()  #mA
        data_5 = str(input_current)  # mA
        data_6 = self.textbox_offset2.text()    # mA 
=======
        #reset all f_R
        if self.flag_fR == False:
            self.flag_fR = True    
            packet_transmission.f_R = 0
            self.calculate_final_fR = 0

        #########TODO: NOTE THAT DATA_1 IS NOT ACTUALLY USED AT AL!!! I NEED TO CHANGE THIS
        data_1 = str(3) #mA
        print(data_1)
        data_3 = str(200) #mA
        data_4 = self.textbox_offset1.text()  #mA
        packet_transmission.offset_1 = float(data_4)
        data_5 = str(200) #mA
        data_6 = self.textbox_offset2.text()    # mA 
        packet_transmission.offset_2 = float(data_6)
            
        data_2 = str(running_frequency) # Hz
>>>>>>> dev
        
        #determines the rotation direction
        # 1 = anti-clockwise
        # 2 = clockwise
        data_7 = rotation_direction
            
        if self.filter_checkbox.isChecked():
            data_8 = 0
        else:
            data_8 = 2
            
        data_9= 0
        data_10 = 1
        
<<<<<<< HEAD
        #########TODO: NOTE THAT DATA_1 IS NOT ACTUALLY USED AT AL!!! I NEED TO CHANGE THIS
        
        packet_transmission.send_function(int(data_1) , data_2, data_3, data_4, data_5, data_6, data_7, data_8, data_9, data_10)
=======
        
        packet_transmission.send_function(data_1 , data_2, data_3, data_4, data_5, data_6, data_7, data_8, data_9, data_10)
        # send all the data to be packed
        packet_transmission.send_transmission_event(this_flag_send=1)            #SET flag for Tx
        packet_transmission.start_flag_send_event(1)
        

        self.status_label.setStyleSheet("color: #32a83a;")
        self.status_label.setText("f<sub>R</sub> begins!...............")
        

        QtCore.QTimer.singleShot(
            3000,
            lambda: self.start_friction_coeff_event(running_frequency, rotation_direction, count_recursion, input_current)
        )

    def start_friction_coeff_event(self, running_frequency = 1,  rotation_direction =  1, count_recursion = 0, input_current = 40):
        global data_1
        global data_2
        global data_3
        global data_4
        global data_5
        global data_6
        global data_7
        global data_8
        global data_9
        global data_10
        
        data_1 = str(10)
        print(data_1)
        data_3 = str(input_current)# mA
        data_4 = self.textbox_offset1.text()  #mA
        data_5 = str(input_current)  # mA
        data_6 = self.textbox_offset2.text()    # mA 
            
        data_2 = str(running_frequency) # Hz
        
        #determines the rotation direction
        # 1 = anti-clockwise
        # 2 = clockwise
        data_7 = rotation_direction
        
        self.status_label.setText(
            f"Recursion {count_recursion} with direction {data_7} and frequency of {data_2}"
        )

            
        if self.filter_checkbox.isChecked():
            data_8 = 0
        else:
            data_8 = 2
            
        data_9= 0
        data_10 = 1
        
        #########TODO: NOTE THAT DATA_1 IS NOT ACTUALLY USED AT AL!!! I NEED TO CHANGE THIS
        
        packet_transmission.send_function(data_1 , data_2, data_3, data_4, data_5, data_6, data_7, data_8, data_9, data_10)
>>>>>>> dev
        # send all the data to be packed
        packet_transmission.send_transmission_event(this_flag_send=1)            #SET flag for Tx
        packet_transmission.start_flag_send_event(1)
        
    
        # send flag for calibration in the thread
        self.worker_DataUpdate.flag_special_event(flag_1=False, flag_2=True)

<<<<<<< HEAD
        self.status_label.setStyleSheet("color: #32a83a;")
        self.status_label.setText("f<sub>R</sub> begins!...............")
        
        QtCore.QTimer.singleShot(int(data_1)*1000, lambda: self.after_stabilise_fR_measurement(count_recursion, running_frequency, input_current))
    
    # def wait(self, val, count_recursion, running_frequency, input_current):
    #     #5 seconds interval
    #     QtCore.QTimer.singleShot(10000, lambda: self.after_stabilise_fR_measurement(val, count_recursion, running_frequency, input_current))
        
    def after_stabilise_fR_measurement(self, count_recursion, running_frequency, input_current):
        self.worker_sleep = SleepThread()
        self.worker_sleep.update_time_signal.connect(lambda value: self.update_time_counter_fR_measurement( value, count_recursion, running_frequency, input_current))
        self.worker_sleep.start()
        
        
    def running_time_10_rotation(self, running_frequency):
        running_time = 10/running_frequency 
        
        return float(running_time)
    
    def update_time_counter_fR_measurement(self, val, count_recursion, running_frequency, input_current):
=======
        
        
        time_delay = 10 * 1000
        QtCore.QTimer.singleShot(time_delay, lambda: self.after_stabilise_fR_measurement(count_recursion, running_frequency, input_current, data_4, data_6))
    

        
    def after_stabilise_fR_measurement(self, count_recursion, running_frequency, input_current, data_4, data_6):
        self.worker_sleep = SleepTimer()
        self.worker_sleep.update_time_signal.connect(lambda value: self.update_time_counter_fR_measurement( value, count_recursion, running_frequency, input_current, data_4, data_6))
        self.worker_sleep.start()
        
    
    def update_time_counter_fR_measurement(self, val, count_recursion, running_frequency, input_current, data_4, data_6):
>>>>>>> dev
        """
        Update the time counter and process data during friction coefficient measurement.

        This method is called repeatedly during a friction coefficient experiment.
        It performs the following operations:

        1. Updates the LCD display with the current value `val`.
        2. Enables or disables the send/start/stop buttons depending on whether the
        measurement thread is running (`val != 0`) or stopped (`val == 0`).
        3. Resets internal flags via `worker_DataUpdate` to ensure data integrity.
        4. Computes the mean currents and Hall voltages from accumulated measurements.
        5. Calculates torque and angular velocity for the current measurement step
        and stores them in `calculated_torque` and `calculated_angular_velocity`.
        6. Handles recursion logic:
        - For `count_recursion < 10`: negative frequency (anticlockwise rotation)
        - For `10 <= count_recursion < 19`: positive frequency (clockwise rotation)
        - Increments `running_frequency` and updates `rotation_direction`.
        7. Recursively triggers the next measurement step via 
        `start_friction_coeff_event`.
        8. When the last measurement (`count_recursion == 19`) is reached:
        - Computes the final friction coefficient slope using `np.polyfit`.
        - Re-enables the buttons for user interaction.

        :param val: Current measurement value from the thread (0 if stopped, non-zero if running).
        :type val: float or int
        :param count_recursion: Index of the current measurement step.
        :type count_recursion: int
        """

        self.lcdNumber.display(val)

<<<<<<< HEAD
        if val != 0:
      
            self.button_send.setDisabled(True)
            self.button_start.setDisabled(True)
            self.button_stop.setDisabled(False)

        elif val== 0:  #when val is 0 and the thread is stops already         
=======
        if val != 0.0:   # means val != 0 and val != 0.1
            self.button_send.setDisabled(True)
            self.button_start.setDisabled(True)
            self.button_stop.setDisabled(True)

        elif val == 0.0:     # means val == 0 or val == 0.1     
>>>>>>> dev
            #reset flags (so that accumulate does not change here )(data integrity reason here)
            self.worker_DataUpdate.flag_special_event(False, False)
            
            
<<<<<<< HEAD
            self.mean_current1_fR = np.mean(self.accumulate_current_1[1000:])
            self.mean_current2_fR= np.mean(self.accumulate_current_2[1000:])
            self.mean_hall1_fR = np.mean(self.accumulate_hall_1[1000:])
            self.mean_hall2_fR= np.mean(self.accumulate_hall_2[1000:])
            
            self.calculated_torque[count_recursion] = self.calculate_torque_fR(self.mean_current1_fR, self.mean_current2_fR, self.mean_hall1_fR, 
                                                    self.mean_hall2_fR)
            
            
            if 0 <= count_recursion < 19:
                if  0 <= count_recursion < 10:
                    ### anticlockwise first 
                        
                        
                    #negative frequency
                    self.calculated_angular_velocity[count_recursion] = -self.calculate_radial_frequency(running_frequency)
                    running_frequency+= 1       #Hz
                        
                        
                    
                    rotation_direction = 1 
                elif 10 <=  count_recursion  < 19:
                    #positive frequency
                    self.calculated_angular_velocity[count_recursion] = self.calculate_radial_frequency(running_frequency)
                    ### increment by one
                    running_frequency+= 1
                    
                    if count_recursion == 10:
                        #reset to 1Hz (since the negative frequency part, i.e. anti-clockwise part, is done)
                        running_frequency = 1
                    
                    rotation_direction = 2
                count_recursion = count_recursion + 1
                print("recursion is", count_recursion)
                if count_recursion == 1:  #Hz
                    input_current = 40 #mA
                elif count_recursion == 2:
                    input_current = 45 
                elif running_frequency == 3:
                    input_current = 50
                elif running_frequency == 4:
                    input_current = 50
                elif running_frequency == 5:
                    input_current = 60 
                elif running_frequency == 6:
                    input_current = 60
                elif running_frequency == 7:
                    input_current = 70
                elif running_frequency == 8:
                    input_current = 80
                elif running_frequency == 9:
                    input_current = 90
                
                ###recursion to the main function occurs
                print("running frquency is ", running_frequency)
                self.start_friction_coeff_event(running_frequency, rotation_direction, count_recursion, input_current)
            
            ### measurement is done
            elif count_recursion == 19:
                # calculate the last angular velocity
                self.calculated_angular_velocity[count_recursion] = self.calculate_radial_frequency(running_frequency)
                

                #calculate final fR
                self.calculate_final_fR =  np.polyfit(self.calculated_angular_velocity, self.calculated_torque, 1)
=======
            self.current1_fR = self.accumulate_current_1[1000:]
            self.current2_fR= self.accumulate_current_2[1000:]
            self.hall1_fR = self.accumulate_hall_1[1000:]
            self.hall2_fR= self.accumulate_hall_2[1000:]
            

            if 0 <= count_recursion < 20:
                
                print("recursion is", count_recursion)

                # Determine sign of angular velocity
                if count_recursion < 10:  # positive / anticlockwise sweep
                    angular_velocity = np.mean(packet_transmission.calculate_radial_frequency(running_frequency))
                    rotation_direction = 1
                else:  # negative / clockwise sweep
                    angular_velocity = -np.mean(packet_transmission.calculate_radial_frequency(running_frequency))
                    rotation_direction = 2

                # Calculate torque and phase
                torque_values, phase_values = packet_transmission.calculate_torque_fR(
                    self.current1_fR, self.current2_fR, self.hall1_fR, self.hall2_fR, data_4, data_6
                )

                # Store results
                self.calculated_angular_velocity[count_recursion] = angular_velocity
                self.calculated_torque[count_recursion] = np.mean(torque_values)
                self.mean_phase[count_recursion] = np.mean(phase_values)
                self.standard_torque[count_recursion] = np.std(torque_values)
                self.standard_mean_torque[count_recursion] = np.std(torque_values) / np.sqrt(len(torque_values))
                self.standard_phase[count_recursion] = np.std(phase_values)
                self.standard_mean_phase[count_recursion] = np.std(phase_values) / np.sqrt(len(phase_values))

                # Update running frequency and rotation direction
                if count_recursion == 9:  # switch from positive to negative sweep
                    running_frequency = 1
                else:
                    running_frequency += 1

                count_recursion += 1
                
                                
                                #mapping for current {count_recursion:input_current}
                current_map = {
                    1: 50,  2: 50,  3: 50,  4: 50,  5: 60,
                    6: 60,  7: 70,  8: 80,  9: 90,
                    10: 40, 11: 50, 12: 50, 13: 50, 14: 50,
                    15: 60, 16: 60, 17: 70, 18: 80, 19: 90
                }

                input_current = current_map.get(count_recursion, 0)  # default 0 if not found

                    
                ###recursion to the main function occurs
                print("running frquency is ", running_frequency)
                self.start_friction_coeff_event_initiation(running_frequency, rotation_direction, count_recursion, input_current)
            
            ### measurement is done
            elif count_recursion == 20:
                # Combine the two arrays column-wise
                data_to_save = np.column_stack(( self.calculated_angular_velocity, self.calculated_torque, self.standard_mean_torque,
                                                self.standard_torque, self.mean_phase, self.standard_mean_phase, self.standard_phase))
                
                
                header_text = "angular_velocity  [rad/s];mean_torque [Nm];std_mean_torque [Nm];std_torque [Nm];mean_phase [rad];std_mean_phase [rad];std_phase [rad]"
                
                dir_fr_results = os.path.join(self.project_root, "files", "results_fr.csv")
                if os.path.exists(dir_fr_results):
                    os.remove(dir_fr_results)
                    print(dir_fr_results)
                # Save to CSV      
                np.savetxt(dir_fr_results, data_to_save, delimiter=";", comments="", fmt="%.18f", header=header_text)
                #calculate final fR
                self.calculate_final_fR =  np.polyfit(self.calculated_angular_velocity, self.calculated_torque, 1)
                self.popout_window(4)
                
                packet_transmission.fr1, packet_transmission.fr0 = self.calculate_final_fR
                
                #change textbox in the bottom of the GUI 
                self.k_b_label.setText(
                    f"k<sub>b1</sub> = {packet_transmission.k_b_1}&nbsp;&nbsp;&nbsp;"
                    f"k<sub>b2</sub> = {packet_transmission.k_b_2}&nbsp;&nbsp;&nbsp;"
                    f"K = {packet_transmission.CALIBRATION_FACTOR}&nbsp;&nbsp;&nbsp;"
                    f"f<sub>R</sub> equation = {np.poly1d(self.calculate_final_fR)}"
                )
                
                #refresh the graph
                self.plot_object = PlotWindow()
                self.plot_object.refresh_graph()
>>>>>>> dev
                
                #enable the button again
                self.button_send.setDisabled(False)
                self.button_start.setDisabled(False)
                self.button_stop.setDisabled(True)
<<<<<<< HEAD
    
                
            
                
    def calculate_torque_fR(self, current_1, current_2, hall_1, hall_2):
        """
        Calculate the torque based on Hall sensor and current measurements.

        This method computes the applied torque using the phase difference between
        the magnetic field (from the current coils) and the magnet (from
        the Hall sensors). It applies calibration factors, coil constants,
        and the dipole moment to convert the measured currents to torque.

        :param current_1: First component of the current measurement.
        :type current_1: float or np.ndarray
        :param current_2: Second component of the current measurement.
        :type current_2: float or np.ndarray
        :param hall_1: First Hall sensor reading.
        :type hall_1: float or np.ndarray
        :param hall_2: Second Hall sensor reading.
        :type hall_2: float or np.ndarray

        :return: Calculated torque.
        :rtype: float or np.ndarray
        """
        
        global data_4, data_6
        offset_1 =float(data_4)
        offset_2 =float(data_6)
        
        #magnetic field angle - magnet angle
        phase_difference = np.arctan2(current_2, current_1) - np.arctan2(hall_2, hall_1)

        power_of_2 = np.power((current_1 - offset_1), 2) + np.power((current_2 - offset_2), 2)
        
        magnitude_current = np.sqrt(power_of_2)
        
        total_torque = (
        self.CALIBRATION_FACTOR                # dimensionslos
        * self.DIPOLE_MOMENT                    # [A·m²]
        * self.COIL_CONSTANT                    # [T/A]
        * magnitude_current / 1000         # mA; -> A, [A]
        * np.sin(phase_difference)   # dimensionless
        )
        
        return total_torque
    
    def calculate_radial_frequency(self, running_frequency):
        
        return float(2 * np.pi * running_frequency)
        
>>>>>>> Stashed changes
=======
                
                
                if val == 0:
                    #enable the button again
                    self.button_send.setDisabled(False)
                    self.button_start.setDisabled(False)
                    self.button_stop.setDisabled(True)
                
        if val == 0:
            #enable the button again
            self.button_send.setDisabled(False)
            self.button_start.setDisabled(False)
            self.button_stop.setDisabled(False)
    
    

>>>>>>> dev

    def stop_button_push_event(self):
        
        ########################### gives -500mA to the coil 1 and +500mA DC to completely stop the rotation of the magnet 
        global data_1
        global data_2
        global data_3
        global data_4
        global data_5
        global data_6
        global data_7        #checkbox for direction
        global data_8
        global data_9
        global data_10
    
        data_1 = 65534
        data_2 =self.textbox_frequency.text()
        data_2 = packet_transmission.calculate_running_frequency(float(data_2))
        data_3 = 0
        data_4 =  -500
        data_5 = 0 
        data_6 = 500

        
        #from combobox direction
        if self.comboBox_direction.currentText() == "Clockwise":
            data_7 = 2
        elif self.comboBox_direction.currentText() == "Anti-clockwise":
            data_7 = 1
            
        if self.filter_checkbox.isChecked():
            data_8 = 0
        else:
            data_8 = 2
            
        data_10 = 1
        
        
        packet_transmission.send_function(data_1, data_2, data_3, data_4, data_5, data_6, data_7, data_8, data_9, data_10)
        #send all the data to be packed
        packet_transmission.send_transmission_event(1)            #SET flag for Tx
        packet_transmission.start_flag_send_event(1)
        
        
        self.status_label.setText("Stop rotation!!....") 
        
    
    
    def graph_stop_event(self):
        if self.stop_default_state == 1:
            
            self.graph_stop_button.setText("Resume live-graph")
            self.stop_default_state = 2
            self.timer.stop()
            
        elif self.stop_default_state == 2:
            
            self.graph_stop_button.setText("Pause live-graph")
            self.stop_default_state = 1
            self.timer.start()
            
            
    def set_hardware_reset_event(self):
        global data_1, data_2, data_3, data_4, data_5, data_6, data_7, data_8, data_9, data_10
        
        data_9 = 1
        packet_transmission.send_function(data_1, data_2, data_3, data_4, data_5, data_6, data_7, data_8, data_9, data_10)
        packet_transmission.send_transmission_event(1)            #SET flag for Tx
        packet_transmission.start_flag_send_event(1)
        data_9 = 0
        
        self.set_software_reset_event()
        
    
    def set_software_reset_event(self):
        # all the background processes
        if self.p_window_data is not None:
            self.p_window_data.terminate()
            self.p_window_data.join()

        #terminate the other subprocess
        sockets_files.p1.terminate()
        sockets_files.p1.join()

        #stop all the threads
        self.worker_socket.stop()
        self.worker_DataUpdate.stop()
        
        #restart the python script
        os.execv(sys.executable, ['python'] + sys.argv)

    def save_button_event(self):
        filename_saving, _ = QFileDialog.getSaveFileName(self, "Save File", "", "CSV Files (*.csv)")
    
        dir_dummy_csv = os.path.join(self.project_root, "files", "dummy.csv")
        
        if filename_saving:
            data_read = np.loadtxt(dir_dummy_csv, delimiter=';')
            np.savetxt(filename_saving, data_read, delimiter=';')




#different windows with tab 
class TabWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MiR | Mini-Rheometer")
        self.resize(1600, 980)  # width, height
        

        # Create tab widget
        tabs = QTabWidget()

        # Add your classes as tabs
        tabs.addTab(ConstShearGUI(), "Main GUI")
        tabs.addTab( AnalyseWindow(), "Data analyse")
        tabs.addTab(PlotWindow(), "f_R Plot")

        # Set central widget
        self.setCentralWidget(tabs)
        
    def closeEvent(self, event):
        #stop all the background processes

            
        for q in (sockets_files.q_to_process, sockets_files.q_to_graph, sockets_files.q_to_csv):
            q.close()
            q.join_thread()


        #terminate the other subprocess
        sockets_files.p1.terminate()
        sockets_files.p1.join()

        #stop all the threads
        
        self.main_window = ConstShearGUI()
        self.main_window.worker_socket.stop()
        self.main_window.worker_DataUpdate.stop()
        
        #DELETE THE GODDAMN FILE 
        try:
            os.remove("dummy.csv")
        except OSError as e:
            print(f"Error deleting file: {e}")
            
        event.accept()



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
<<<<<<< HEAD
<<<<<<< Updated upstream
=======
        # disable experiment combobox initially
        self.choose_experiment_comboBox.setEnabled(False)

=======
        
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
>>>>>>> dev

        # connect electronic combobox signal at init
        self.choose_electronic_comboBox.currentIndexChanged.connect(self.enable_choose_experiment)

        # connect experiment combobox signal at init
<<<<<<< HEAD
>>>>>>> Stashed changes
=======
>>>>>>> dev
        self.choose_experiment_comboBox.activated.connect(self.choose_window)
        
        self.choose_com_comboBox.activated.connect(self.choose_com_port)
        
    def choose_com_port(self):
        
        choose_com_text = str(self.choose_com_comboBox.currentText())
        
        if choose_com_text == "Pick Virtual COM Port for USB":
            self.choose_electronic_comboBox.setEnabled(False)
        else:
            self.choose_electronic_comboBox.setEnabled(True)
            sockets_files.port_name_setter(choose_com_text)
        

    
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
<<<<<<< HEAD
<<<<<<< Updated upstream
                data_10 = 1
                q_get_mir_mode.put(data_10)
                self.shear_constant_mode_window.show()
                self.close()
    
=======
            
            q_get_mir_mode.put(1)
            # declare the window first without showing it 
            self.shear_constant_mode_window = ConstShearGUI()
            self.shear_constant_mode_window.show()
            self.close()      
        
>>>>>>> Stashed changes

=======
            # declare the window first without showing it 
            # self.shear_constant_mode_window = ConstShearGUI()
            # self.shear_constant_mode_window.showNormal()
            # self.close()      
            
            self.csr_window = TabWindow()    
            self.csr_window.showMaximized()
            
        elif mode == "Creep experiment":
            self.ct_window = CreepTestGUI()
            self.ct_window.showMaximized()
            
            
        self.close()
>>>>>>> dev

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
    
