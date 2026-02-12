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
from .pid_output import Ui_Title
from .change_coeff_tab import SendPIDCoeff

import socket_GUI.sockets_files as sockets_files
from socket_GUI.sockets_files import q_to_graph

import packet_transmission as packet_transmission



# GLOBAL VARIABLES
data_mutex = QMutex()


# function receiving data through pipe from another thread
class DataUpdate(QThread):
    """
    Thread class responsible for receiving, processing, and managing ADC data streams for Hall sensors and current sensors in real-time.

    This class reads incoming data packets from a queue, converts raw ADC values into meaningful voltage and current signals, 
    applies calibration and normalization, and computes derived quantities such as magnetic field angles and phase differences.

    The processed data is stored in a separate worker class (`StoreArrayGraph`) for visualization or further analysis.

    Attributes
    ----------
    running : bool
        Controls whether the thread continues running.
    flag_calibrate : bool
        Enables calibration mode when True.
    flag_fR_measurement : bool
        Enables frequency response measurement mode when True.
    flag_normalise : bool
        Enables signal normalization when True.
    accumulate_hall_1, accumulate_hall_2 : np.ndarray or None
        Accumulated Hall sensor data for calibration.
    accumulate_current_1, accumulate_current_2 : np.ndarray or None
        Accumulated current sensor data for calibration.
    total_hall_1, total_hall_2 : np.ndarray or None
        Total calibrated Hall sensor values.
    total_current_1, total_current_2 : np.ndarray or None
        Total calibrated current sensor values.
    v1_slice, v2_slice : np.ndarray
        Processed voltage values from Hall sensors.
    i1_slice, i2_slice : np.ndarray
        Processed current values from input coils.
    bytes_to_process : np.ndarray
        Buffer for raw incoming ADC data (uint16).
    angle_permanent_magnet_val : np.ndarray
        Computed angular position of the permanent magnet based on Hall sensor data.
    angle_magnetic_field_val : np.ndarray
        Computed angular position of the magnetic field based on current data.
    phase_difference_val : np.ndarray
        Phase difference between magnetic field and permanent magnet angles.
    worker_array_setter : StoreArrayGraph
        Object responsible for storing processed arrays for visualization.
    """

    def __init__(self, main_window_ref=None):
        super().__init__()
        self.main_window_ref = main_window_ref
        self.running = True
        self.flag_calibrate = False

        self.flag_fR_measurement = False
        self.accumulate_hall_1 = 0.0
        self.accumulate_hall_2 = 0.0
        self.accumulate_current_1 = 0.0
        self.accumulate_current_2 = 0.0

        # for normalise properties purposes
        self.worker_normalise_properties = packet_transmission.VoltageNormaliseCoefficient()
        self.amplitude_voltage_1 = 0.0
        self.zero_offset_voltage_1 = 0.0
        self.amplitude_voltage_2 = 0.0
        self.zero_offset_voltage_2 = 0.0

        self.flag_normalise = False
        self.flag_normalise_measurement = False

        self.total_hall_1 = None
        self.total_hall_2 = None
        self.total_current_1 = None
        self.total_current_2 = None

        self.v1_slice = np.array([], dtype=np.uint16)
        self.v2_slice = np.array([], dtype=np.uint16)
        self.i1_slice = np.array([], dtype=np.uint16)
        self.i2_slice = np.array([], dtype=np.uint16)
        self.phase_diff_slice = np.array([], dtype=np.float32)

        self.bytes_to_process = np.array([], dtype=np.uint16)  # Empty NumPy array for incoming data

        self.angle_permanent_magnet_val = np.array([], dtype=np.float32)
        self.angle_magnetic_field_val = np.array([], dtype=np.float32)
        self.phase_difference_val = np.array([], dtype=np.float32)

        self.worker_array_setter = packet_transmission.StoreArrayGraph()
        self.worker_kb_property = packet_transmission.kbCoefficient()

    def run(self):

        """
        Main execution loop for the data update thread.

        This function continuously retrieves data from the `q_to_graph` queue, processes
        it, and updates the corresponding arrays. The data processing steps include:

        1. **Reshaping and Conversion**: Raw 16-bit ADC samples are reshaped into  voltage and current columns.
        2. **Signal Conversion**: ADC counts are converted to voltages (Hall sensors)  and currents (input coils).
        3. **Calibration**: If enabled, calibration routines adjust sensor outputs.
        4. **Normalization**: Optionally normalizes signal amplitude and offsets.
        5. **Angle Computation**: Calculates angular positions using `arctan2` for both magnetic field and permanent magnet signals.
        6. **Phase Difference Calculation**: Determines the phase difference between the two computed angles.
        7. **Data Storage**: Updates the `worker_array_setter` with processed arrays.

        The loop continues until the `running` flag is set to False.

        Notes
        -----
        - Thread synchronization is managed using `data_mutex` to ensure data integrity.
        - Data from the queue (`q_to_graph`) must have a multiple of 4 samples,
          corresponding to [v1, v2, i1, i2].
        """

        data_from_pipe = []  # creating a list here because data from pipe is a list
        while self.running:
            data_from_pipe = q_to_graph.get()
            if not self.running:
                break
            if data_from_pipe:
                self.bytes_to_process = np.array(data_from_pipe, dtype=float)

                num_columns = 5  # v1, v2, i1, i2, phase_diff

                trimmed_size = len(self.bytes_to_process) - (len(self.bytes_to_process) % num_columns)
                self.bytes_to_process = self.bytes_to_process[:trimmed_size]

                if self.bytes_to_process.size == 0:
                    return

                reshaped_data = self.bytes_to_process.reshape(-1, num_columns)

                self.v1_slice = reshaped_data[:, 0]
                self.v2_slice = reshaped_data[:, 1]
                self.i1_slice = reshaped_data[:, 2]
                self.i2_slice = reshaped_data[:, 3]
                self.phase_diff_slice = reshaped_data[:, 4]
                print(self.phase_diff_slice)

                data_mutex.lock()

                # Hall Sensors
                # self.v1_slice = -packet_transmission.change_adc_hall(self.v1_slice)  # convert col1 (in V)
                # self.v2_slice = packet_transmission.change_adc_hall(self.v2_slice)  # convert col2 (in V)
                

                # Current
                self.i1_slice = -packet_transmission.change_current_adc(self.i1_slice)  # convert col3 (in mA)
                self.i2_slice = packet_transmission.change_current_adc(self.i2_slice)  # convert col4 (in mA)

                # calibration for current sensor
                self.i1_slice = packet_transmission.calibration_input_coil_1(self.i1_slice)
                self.i2_slice = packet_transmission.calibration_input_coil_2(self.i2_slice)


                # # calibration for  hall sensors
                # self.v1_slice = packet_transmission.calibrated_hall_sensors1(self.worker_kb_property.k_b_1,
                #                                                              self.v1_slice, self.i1_slice / 1000)
                # self.v2_slice = packet_transmission.calibrated_hall_sensors2(self.worker_kb_property.k_b_2,
                #                                                              self.v2_slice, self.i2_slice / 1000)

                # this is normalising step (still do not know whether I want to do it immidiately or not)
                if self.flag_normalise:
                    self.v1_slice = (self.v1_slice - self.worker_normalise_properties.zero_offset_voltage_1) / self.worker_normalise_properties.amplitude_voltage_1
                    self.v2_slice = ( self.v2_slice - self.worker_normalise_properties.zero_offset_voltage_2) / self.worker_normalise_properties.amplitude_voltage_2

                # Calibrate process starts
                # measurement fR process starts
                # measurement to determine the normalising parameters (for first time rotation)
                if self.flag_calibrate or self.flag_fR_measurement or self.flag_normalise_measurement:
                    self.accumulate_data_function(self.v1_slice, self.v2_slice, self.i1_slice, self.i2_slice)


                #######################################################################################################
                self.angle_permanent_magnet_val = np.arctan2(self.v2_slice, self.v1_slice)
                self.angle_magnetic_field_val = np.arctan2(self.i2_slice, self.i1_slice)

                self.angle_permanent_magnet_val = np.unwrap(self.angle_permanent_magnet_val)
                self.angle_magnetic_field_val = np.unwrap(self.angle_magnetic_field_val)
                #######################################################################################################
                self.phase_difference_val = self.phase_diff_slice

                ##send to setter class
                self.worker_array_setter.v1_slice = self.v1_slice
                self.worker_array_setter.v2_slice = self.v2_slice
                self.worker_array_setter.i1_slice = self.i1_slice
                self.worker_array_setter.i2_slice = self.i2_slice

                self.worker_array_setter.angle_permanent_magnet_val = self.angle_permanent_magnet_val
                self.worker_array_setter.angle_magnetic_field_val = self.angle_magnetic_field_val

                self.worker_array_setter.phase_difference_val = self.phase_difference_val

                data_mutex.unlock()

                # Clear queue after processing
                self.bytes_to_process = np.array([], dtype=np.uint16)

    def accumulate_data_function(self, store_array1_calibrate, store_array2_calibrate, store_array3_calibrate,
                                 store_array4_calibrate):

        self.accumulate_hall_1 = np.append(self.total_hall_1, store_array1_calibrate)
        self.accumulate_hall_2 = np.append(self.total_hall_2, store_array2_calibrate)
        self.accumulate_current_1 = np.append(self.total_current_1, store_array3_calibrate)
        self.accumulate_current_2 = np.append(self.total_current_2, store_array4_calibrate)

        self.main_window_ref.set_constant(self.accumulate_hall_1,
                                          self.accumulate_hall_2,
                                          self.accumulate_current_1,
                                          self.accumulate_current_2)

    def flag_special_event(self, flag_calibrate, flag_fR_measurement, flag_normalise_measurement):

        # Set flag for calibration
        self.flag_calibrate = flag_calibrate
        # Set flag for fR measurement
        self.flag_fR_measurement = flag_fR_measurement
        # Set flag for normalise measurement event
        self.flag_normalise_measurement = flag_normalise_measurement


    def flag_normalise_event(self, flag_input):

        self.flag_normalise = flag_input

    def stop(self):
        self.running = False


class SleepTimer(QObject):
    """
    A countdown timer that emits time updates at fixed intervals using PyQt signals.

    This class implements a simple countdown timer based on `QTimer`. It periodically
    emits the remaining time (in seconds) until the countdown reaches zero. The timer
    is intended to be used in GUI applications where a background countdown must update
    a display or trigger an event when completed.

    Attributes
    ----------
    update_time_signal : pyqtSignal(float)
        Signal emitted with the remaining time in seconds (rounded to one decimal place)
        after each timer tick.
    remaining : float
        The remaining time in seconds. Initialized from the global variable `data_1`.
    timer : QTimer
        Internal Qt timer that triggers the `_tick` method every 100 milliseconds.

    Methods
    -------
    start()
        Starts the countdown timer.
    stop()
        Stops the countdown timer.
    _tick()
        Decrements the remaining time by 0.1 seconds per tick, emits updates via
        `update_time_signal`, and calls `packet_transmission.running_time_flag_setter(0)`
        when the countdown reaches zero.

    Notes
    -----
    - The initial countdown value is taken from the global variable `data_1`.
    - Each tick occurs every 100 ms (0.1 s).
    - When the countdown completes, the timer stops automatically.
    """
    update_time_signal = pyqtSignal(float)  # emit float countdown values

    def __init__(self):
        super().__init__()
        self.worker_remaining = packet_transmission.TxData()
        self.remaining = float(self.worker_remaining.data_1) # get local_data_1 from global
        self.worker_flag_run_time = packet_transmission.RunningTimeFlag()
        self.worker_reset_current_time = packet_transmission.DownSampleSpecificFlag()
        self.timer = QTimer(self)
        self.timer.setInterval(100)  # 100 ms per tick
        self.timer.timeout.connect(self._tick)
        

    def start(self):
        self.timer.start()

    def stop(self):
        self.timer.stop()

    def _tick(self):
        self.remaining -= 0.1
        if self.remaining >= 0.0:
            self.update_time_signal.emit(round(self.remaining, 1))
        else:
            self.timer.stop()
            self.worker_flag_run_time.flag_running_time = False

            # packet_transmission.running_time_flag_setter(0)

class SocketThread(QThread):
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.running = True

    def run(self):
        if self.running:
            sockets_files.thread_start()


    def stop(self):
        self.running = False
    


class PIDOutput(QMainWindow, Ui_Title):

    def __init__(self):
        super().__init__()
        self.setupUi(self)
        
        if hasattr(QtCore.Qt, 'AA_EnableHighDpiScaling'):
            QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
        if hasattr(QtCore.Qt, 'AA_UseHighDpiPixmaps'):
            QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)

        # get absolute path of project root (folder containing 'src' and 'pics')
        self.project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

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
        self.time_axis =  [i * 0.0001 for i in range(1000)]
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
        
        ####### flag init ##############################################################################
        
        
        #for receive thread
        self.worker_flag_run_time = packet_transmission.RunningTimeFlag()
        #for tx data
        self.worker_data_block = packet_transmission.TxData()
        #for transmitting thread
        self.worker_flag_send = packet_transmission.TxFlag()
        #for data straight from graph
        self.worker_getter_graph = packet_transmission.StoreArrayGraph()
        #for fr setter getter
        self.worker_fr_property = packet_transmission.fRCoefficients()
        
        
        #for k_b setter getter
        self.worker_k_b_property = packet_transmission.kbCoefficient()
        #for downsampling purposes
        self.worker_downsample_property = packet_transmission.DownSampleSpecificFlag()
        self.tot_average = self.worker_downsample_property.tot_average
        self.time_increment = self.worker_downsample_property.current_time

        #for normalise properties purposes
        self.worker_normalise_properties = packet_transmission.VoltageNormaliseCoefficient()
        
        
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
        self.worker_get = packet_transmission.fRCoefficients()
        packet_transmission.CALIBRATION_FACTOR = self.worker_get.CALIBRATION_FACTOR # torque calibration no units (K)
    
        #########################################################################################
        #placeholder text for textboxes################################################
        
        
        
             #### connected functions for button
        self.button_send.clicked.connect(self.send_parameter_event)
        self.button_send.clicked.connect(lambda value: self.popout_window(1))
        

        self.actionHardware_reset.triggered.connect(self.set_hardware_reset_event)
        self.actionSoftware_restart.triggered.connect(self.set_software_reset_event)
        
        self.button_auto_range.clicked.connect(self.auto_range_event)
        
        self.save_button.clicked.connect(self.save_button_event)
    
        self.button_rotate.clicked.connect(self.rotate_magnet_event)

        #Start backend serial lines
        
        # self.worker_socket = SocketThread()
        # self.worker_DataUpdate = DataUpdate(self)
        
        # self.worker_socket.start()
        # self.worker_DataUpdate.start()
        
        
        #combobox for live graph mode
        self.select_mode_comboBox.activated.connect(self.change_graph)
    
        # Call the handler immediately with the current index for the live graph
        self.change_graph()
        #same as above but for time interval change
        self.timeInterval_comboBox.activated.connect(self.change_graph)
        
        
        
    def change_graph(self):
        
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
                sockets_files.TOT_COUNT_ACCUMULATE_RECV_IN_1_SEC = sockets_files.TOT_COUNT_ACCUMULATE_RECV_IN_1_SEC_FRONTEND
                self.time_axis = [i * 0.0001 for i in range(1000)]
            elif time_interval_var_int == 500:
                sockets_files.TOT_COUNT_ACCUMULATE_RECV_IN_1_SEC = 5*sockets_files.TOT_COUNT_ACCUMULATE_RECV_IN_1_SEC_FRONTEND
                self.time_axis = [i * 0.0001 for i in range(5*1000)]
            elif time_interval_var_int == 1000:
                sockets_files.TOT_COUNT_ACCUMULATE_RECV_IN_1_SEC = 10*sockets_files.TOT_COUNT_ACCUMULATE_RECV_IN_1_SEC_FRONTEND
                self.time_axis = [i * 0.0001 for i in range(10*1000)]
            
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
                sockets_files.TOT_COUNT_ACCUMULATE_RECV_IN_1_SEC = sockets_files.TOT_COUNT_ACCUMULATE_RECV_IN_1_SEC_FRONTEND
                self.time_axis = [i * 0.0001 for i in range(1000)]
            elif time_interval_var_int == 500:
                sockets_files.TOT_COUNT_ACCUMULATE_RECV_IN_1_SEC = 5*sockets_files.TOT_COUNT_ACCUMULATE_RECV_IN_1_SEC_FRONTEND
                self.time_axis = [i * 0.0001 for i in range(5*1000)]
            elif time_interval_var_int == 1000:
                sockets_files.TOT_COUNT_ACCUMULATE_RECV_IN_1_SEC = 10*sockets_files.TOT_COUNT_ACCUMULATE_RECV_IN_1_SEC_FRONTEND
                self.time_axis = [i * 0.0001 for i in range(10*5000)]
                
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
                sockets_files.TOT_COUNT_ACCUMULATE_RECV_IN_1_SEC = sockets_files.TOT_COUNT_ACCUMULATE_RECV_IN_1_SEC_FRONTEND
                self.time_axis = [i * 0.0001 for i in range(1000)]
            elif time_interval_var_int == 500:
                sockets_files.TOT_COUNT_ACCUMULATE_RECV_IN_1_SEC = 5*sockets_files.TOT_COUNT_ACCUMULATE_RECV_IN_1_SEC_FRONTEND
                self.time_axis = [i * 0.0001 for i in range(5*1000)]
            elif time_interval_var_int == 1000:
                sockets_files.TOT_COUNT_ACCUMULATE_RECV_IN_1_SEC = 10*sockets_files.TOT_COUNT_ACCUMULATE_RECV_IN_1_SEC_FRONTEND
                self.time_axis = [i * 0.0001 for i in range(10*1000)]
                
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
                

    def start_normalise_event(self):
        if not self.worker_DataUpdate.flag_normalise:
            self.worker_DataUpdate.flag_normalise_event(True)
        else:
            self.worker_DataUpdate.flag_normalise_event(False)

        print(self.worker_DataUpdate.flag_normalise)
                

    def graph_update_sensors(self):
        global data_mutex

        # data_mutex.lock()

        self.curve_v1.setData(self.time_axis,self.worker_getter_graph.v1_slice)
        self.curve_v2.setData(self.time_axis,self.worker_getter_graph.v2_slice)
        
        self.curve_i1.setData(self.time_axis,self.worker_getter_graph.i1_slice)
        self.curve_i2.setData(self.time_axis,self.worker_getter_graph.i2_slice)
        
        # data_mutex.unlock()
        #
        
    def auto_range_event(self):
        
        self.plot1.enableAutoRange(axis='y', enable=True)
        self.plot2.enableAutoRange(axis='y', enable=True)
        # self.plot1.enableAutoRange(axis='x', enable=True)
        # self.plot2.enableAutoRange(axis='x', enable=True)
        

    def graph_update_angle(self):
        global data_mutex
        data_mutex.lock()
        
        self.curve_sigma_m.setData(self.time_axis, self.worker_getter_graph.angle_permanent_magnet_val)
        self.curve_sigma_b.setData(self.time_axis, self.worker_getter_graph.angle_magnetic_field_val)
        
        data_mutex.unlock()
    def graph_phase_difference(self):
        global data_mutex
        data_mutex.lock()
        
        self.curve_phase_difference.setData(self.time_axis, self.worker_getter_graph.phase_difference_val)
        
        data_mutex.unlock()
    def send_parameter_event(self):
        
        
        """ 
        Send data event to STM32H7575XI MCU/backend such as the amplitude and offsets for the two pair of coils, the frequency of DAC, the 
        rotation of the magnet and whether to use FIR filtering or not. 
        
        """

        ############################# send data to setter getter ######################################
        self.worker_data_block.data_1 = 65534
    
        self.worker_data_block.data_2  = self.textbox_shear_rate.text()
        
        
        #from combobox direction
        if self.comboBox_direction.currentText() == "Clockwise":
            self.worker_data_block.data_7 = 2
            data_7 = self.worker_data_block.data_7
        elif self.comboBox_direction.currentText() == "Anti-clockwise":
            self.worker_data_block.data_7 = 1
            data_7 = self.worker_data_block.data_7
            
        if self.filter_checkbox.isChecked():
            self.worker_data_block.data_8 = 0
        else:
            self.worker_data_block.data_8 =  2
        
        #for data 10
        self.worker_data_block.data_10 = 1 
        self.worker_data_block.data_11 = self.textbox_shear_stress.text()
        ######################################################################################
        ##enabled stop rotating button 
        self.button_stop.setDisabled(False)
        
        ##send all data to microcontroller
        #activate flag
        self.worker_flag_send.flag_tx = True
        

        self.status_label.setStyleSheet("color: #32a83a;")
        self.status_label.setText("Data sent!")
        
        
    def rotate_magnet_event(self):
        ############################# send data to setter getter ######################################
        self.worker_data_block.data_1 = 65534
        
        #frequency rotor to 3Hz
        self.worker_data_block.data_2  = 3
        
        
        self.worker_data_block.data_current = 200,0,200,0
        
        #from combobox direction
        if self.comboBox_direction.currentText() == "Clockwise":
            self.worker_data_block.data_7 = 2
            data_7 = self.worker_data_block.data_7
        elif self.comboBox_direction.currentText() == "Anti-clockwise":
            self.worker_data_block.data_7 = 1
            data_7 = self.worker_data_block.data_7
            
        if self.filter_checkbox.isChecked():
            self.worker_data_block.data_8 = 0
        else:
            self.worker_data_block.data_8 =  2
            
            
        #for data 10
        #HERE IS THE MOST IMPORTANT THING!!!
        self.worker_data_block.data_10 = 2
        self.worker_data_block.data_11 = self.textbox_shear_stress.text()

        ######################################################################################
        ##enabled stop rotating button 
        self.button_stop.setDisabled(False)
        
        ##send all data to microcontroller
        #activate flag
        self.worker_flag_send.flag_tx = True
        

        self.status_label.setStyleSheet("color: #32a83a;")
        self.status_label.setText("Rotating!")
        
        
        
    def set_hardware_reset_event(self):
        
        self.worker_data_block.data_9 =  1
        self.worker_flag_send =  True
        self.worker_data_block.data_9 =  0 
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
        # Disable immediately to prevent double clicks
        self.save_button.setEnabled(False)

        # Show the dialog (modal → blocks this function until closed)
        filename_saving, _ = QFileDialog.getSaveFileName(
            self, "Save File", "", "CSV Files (*.csv)"
        )
        
        # Re-enable after dialog is closed (save or cancel)
        self.save_button.setEnabled(True)

        # Continue only if the user selected a file
        if filename_saving:
            dir_dummy_csv = os.path.join(self.project_root, "files", "dummy.csv")

            data_read = np.loadtxt(dir_dummy_csv, delimiter=';')
            np.savetxt(filename_saving, data_read, delimiter=';')

        
    def popout_window(self, arg, calculate_final_fR = 0.0, k_b_1 = 0.0, k_b_2 = 0.0):
        
        msg = QMessageBox()
        
        text = packet_transmission.set_popout_text(arg, calculate_final_fR, k_b_1, k_b_2)
        msg.setText(text)
        
        msg.setIcon(QMessageBox.Question)
        
        msg.exec()
        




#Main windows with different tab 
class TabWindowPID(QMainWindow):
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MiR | Mini-Rheometer")
        self.resize(1600, 980)  # width, height
        

        # Create tab widget
        tabs = QTabWidget()

        # Add your classes as tabs
        tabs.addTab(PIDOutput(), "Main GUI")
        tabs.addTab(SendPIDCoeff(), "PID Tx")
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
        
        self.main_window = PIDOutput()
        self.main_window.worker_socket.stop()
        self.main_window.worker_DataUpdate.stop()
        
        #DELETE THE GODDAMN FILE 
        try:
            os.remove("dummy.csv")
        except OSError as e:
            print(f"Error deleting file: {e}")
            
        event.accept()
    
