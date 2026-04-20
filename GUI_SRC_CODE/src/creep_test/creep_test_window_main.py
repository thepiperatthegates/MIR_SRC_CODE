import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5 import *
from PyQt5.QtCore import QThread, pyqtSignal, QMutex, QRegularExpression, QObject, QTimer, Qt
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QRegularExpressionValidator, QDoubleValidator
import os
from pathlib import Path


import socket_GUI.device_state as device_state

# #fix cache problem with MATHPLOTLIB
os.environ['MPLCONFIGDIR'] = str(Path.home()) +"/.matplotlib/"
import sys

from socket_GUI import serial_backend
from socket_GUI.serial_backend import q_to_graph

from .creep_test_gui import Ui_CreepTestGUI
from analyse_window.analyse_window_main import AnalyseWindow 



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
        
        # ---------------- Operational State ---------------
        self.running = True
        self.flag_calibrate = False
        self.flag_normalise = False
        self.flag_normalise_measurement = False
        self.flag_fR_measurement = False

        # ----------- Accumulators & Totals (Hall/Current) ----------
        self.accumulate_hall_1 = 0.0
        self.accumulate_hall_2 = 0.0
        self.accumulate_current_1 = 0.0
        self.accumulate_current_2 = 0.0

        self.total_hall_1 = None
        self.total_hall_2 = None
        self.total_current_1 = None
        self.total_current_2 = None

        # ---------- Normalization Parameters -----------
        self.amplitude_voltage_1 = 0.0
        self.zero_offset_voltage_1 = 0.0
        self.amplitude_voltage_2 = 0.0
        self.zero_offset_voltage_2 = 0.0

        # ------- Data Buffers (NumPy Arrays) --------
        # Raw Data Slices
        self.v1_slice = np.array([], dtype=np.uint16)
        self.v2_slice = np.array([], dtype=np.uint16)
        self.i1_slice = np.array([], dtype=np.uint16)
        self.i2_slice = np.array([], dtype=np.uint16)
        self.bytes_to_process = np.array([], dtype=np.uint16)

        #------------ Calculated Values ------------------------
        self.angle_permanent_magnet_val = np.array([], dtype=np.float32)
        self.angle_magnetic_field_val = np.array([], dtype=np.float32)
        self.phase_difference_val = np.array([], dtype=np.float32)
        self.actual_torque = np.array([], dtype=np.float32)

        # ------ Helper Workers --------------------------------
        self.worker_normalise_properties = device_state.VoltageNormaliseCoefficient()
        self.worker_array_setter = device_state.StoreArrayGraph()
        self.worker_kb_property = device_state.kbCoefficient()
        
        
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
        
        num_columns = 6  # v1, v2, i1, i2, phase_diff
                
        while self.running:
            data_from_pipe = q_to_graph.get()
            if not self.running:
                break
            if data_from_pipe:
                self.bytes_to_process = data_from_pipe  # now changes to np array so we can work with it better

                trimmed_size = len(self.bytes_to_process) - (len(self.bytes_to_process) % num_columns)
                self.bytes_to_process = self.bytes_to_process[:trimmed_size]

                if len(self.bytes_to_process) == 0:
                    return
                reshaped_data = np.array(self.bytes_to_process).reshape(-1, num_columns)
                reshaped_data = reshaped_data.astype(float)

                self.v1_slice = reshaped_data[:, 0]
                self.v2_slice = reshaped_data[:, 1]
                self.i1_slice = reshaped_data[:, 2]  # STIMMT
                self.i2_slice = reshaped_data[:, 3]  # STIMMT
                self.phase_diff_slice = reshaped_data[:, 4]         # phase diff [rad]
                self.torque_slice = reshaped_data[:, 5]         # phase diff [rad]

                data_mutex.lock()

                # Hall Sensors
                #TODO: NEW FIRMWARE ITERACTIONS MUST NOT BE NEGATIVE FOR I1!!!!!!!!!
                # self.v1_slice = -packet_transmission.change_adc_hall(self.v1_slice)  # convert col1 (in V)
                # self.v2_slice = packet_transmission.change_adc_hall(self.v2_slice)  # convert col2 (in V)

                # Current
                self.i1_slice = -device_state.change_current_adc(self.i1_slice)  # convert col3 (in mA)
                self.i2_slice = device_state.change_current_adc(self.i2_slice)  # convert col4 (in mA)

                # calibration for current sensor
                self.i1_slice = device_state.calibration_input_coil_1(self.i1_slice)
                self.i2_slice = device_state.calibration_input_coil_2(self.i2_slice)


                # calibration for  hall sensors
                # self.v1_slice = packet_transmission.calibrated_hall_sensors1(self.worker_kb_property.k_b_1,
                #                                                              self.v1_slice, self.i1_slice / 1000)
                # self.v2_slice = packet_transmission.calibrated_hall_sensors2(self.worker_kb_property.k_b_2,
                #                                                              self.v2_slice, self.i2_slice / 1000)

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
                                
                self.worker_array_setter.actual_torque_val = self.actual_torque


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
        self.worker_remaining = device_state.TxData()
        self.remaining = float(self.worker_remaining.data_1) # get local_data_1 from global
        self.worker_flag_run_time = device_state.RunningTimeFlag()
        self.worker_reset_current_time = device_state.DownSampleSpecificFlag()
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
            
class SleepTimerVector(QObject):
    
    update_time_signal_vector = pyqtSignal(float)
    
    def __init__(self, time_taken):
        super().__init__()
        
        self.remaining = time_taken

        self.timer = QTimer(self)
        self.timer.setInterval(100)
        self.timer.timeout.connect(self._tick)
        
    def start(self):
        self.timer.start()
        
    def stop(self):
        self.timer.stop()
        
    def _tick(self):
        self.remaining -= 0.1
        if self.remaining >= 0.0:
            self.update_time_signal_vector.emit(round(self.remaining, 1))
            
        else:
            self.timer.stop()



class SocketThread(QThread):
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.running = True

    def run(self):
        if self.running:
            serial_backend.thread_start()


    def stop(self):
        self.running = False
    
        

class CreepTestGUI(QMainWindow, Ui_CreepTestGUI):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
            
        self.accumulate_hall_1 = None
        self.accumulate_hall_2 = None
        self.accumulate_current_1 = None
        self.accumulate_current_2 = None


        ################################HINT TYPE###################################################
        
        
        self.textbox_offset_1.setPlaceholderText("Enter +-480mA")
        self.textbox_offset_2.setPlaceholderText("Enter +-480mA")
        
        self.textbox_direction_start_x.setPlaceholderText("X-axis")
        self.textbox_direction_start_y.setPlaceholderText("Y-axis")
        self.button_send_start_vec.setText("I⃗₁")
        self.textbox_direction_end_x.setPlaceholderText("X-axis")
        self.textbox_direction_end_y.setPlaceholderText("Y-axis")
        
        
        self.textbox_input_sampling_rate.setPlaceholderText("Specify the fast sampling rate!")
        self.textbox_input_sampling_time.setPlaceholderText("Counting from 0s")
        
        self.textbox_standard_sampling_rate.setText("100")
        self.textbox_input_sampling_rate.setText("500")
        self.textbox_offset_1.setText("0")
        self.textbox_offset_2.setText("0")
        ################################################################################################

        ################################ SET ICON ###################################################
        # get absolute path of project root (folder containing 'src' and 'pics')
        self.project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        save_icon_path = os.path.join(self.project_root, "pics", "save_icon.ico")
        
        self.save_button.setIcon(QtGui.QIcon(save_icon_path))
        ################################################################################################
        
        
        self.curve_v1 = self.curve_v2 = self.curve_i1 = self.curve_i2 = self.curve_sigma_b = self.curve_sigma_m = None
        self.time_axis =  [i * 0.0001 for i in range(1000)]
        
        
        ################################### connected signals from widget #############################################
        
        self.button_start_acquistion.clicked.connect(self.start_creep_test)
        self.button_start_acquistion.clicked.connect(lambda value: self.popout_window(1))

        self.button_offsets.clicked.connect(self.send_offsets)
        self.button_offsets.clicked.connect(lambda value: self.popout_window(1))

        self.button_send_start_vec.clicked.connect(self.send_start_vector)
        self.button_send_start_vec.clicked.connect(lambda value: self.popout_window(1))


        self.save_button.clicked.connect(self.save_button_event)
        self.graph_stop_button.clicked.connect(self.graph_stop_event)
        self.button_auto_range.clicked.connect(self.auto_range_event)

        self.button_rotate.clicked.connect(self.normalisation_magnet_event)
        
        ####################################################################################
        
        ############################Start backend serial lines##################################################
        
        self.worker_socket = SocketThread()
        self.worker_DataUpdate = DataUpdate(self)

        self.worker_socket.start()
        self.worker_DataUpdate.start()
        self.worker_sleep = None
        self.worker_vector_sleep = None
        ################################################################################################
        
        ####### flag init ##############################################################################
        
        #for receive thread
        self.worker_flag_run_time = device_state.RunningTimeFlag()
        #for tx data
        self.worker_data_block = device_state.TxData()
        #for transmitting thread
        self.worker_flag_send = device_state.TxFlag()
        #for data straight from graph
        self.worker_getter_graph = device_state.StoreArrayGraph()
        #for downsampling purposes
        self.worker_downsample_property = device_state.DownSampleSpecificFlag()
        self.tot_average = self.worker_downsample_property.tot_average
        self.tot_average_specified = self.worker_downsample_property.tot_average_specified
        self.time_increment = self.worker_downsample_property.time_increment
        self.time_increment_specified = self.worker_downsample_property.time_increment_specified
        self.current_time = self.worker_downsample_property.current_time
        
        self.worker_remaining_time = device_state.RemainingTimeForCreepTest()
        self.time_for_resetting_flag = self.worker_remaining_time.time_for_resetting_time
        
        self.start_vector_time = self.worker_remaining_time.start_vector_time
        self.end_vector_time = self.worker_remaining_time.end_vector_time
        
        self.worker_get = device_state.fRCoefficients()
        device_state.CALIBRATION_FACTOR = self.worker_get.CALIBRATION_FACTOR # torque calibration no units (K)

        #for k_b setter getter
        self.worker_k_b_property = device_state.kbCoefficient()
        self.k_b_label.setTextFormat(Qt.RichText)
        self.k_b_label.setText(
            f"k<sub>b1</sub> = {self.worker_k_b_property.k_b_1}&nbsp;&nbsp;&nbsp;"
            f"k<sub>b2</sub> = {self.worker_k_b_property.k_b_2}&nbsp;&nbsp;&nbsp;"
            f"K = {device_state.CALIBRATION_FACTOR}&nbsp;&nbsp;&nbsp;"
        )


        #for normalising properties purposes
        self.worker_normalise_properties = device_state.VoltageNormaliseCoefficient()
        ################################################################################################
        
        ##### combobox for live graph mode
        self.select_mode_comboBox.activated.connect(self.change_graph)
        ##### same as above but for time interval change
        self.timeInterval_comboBox.activated.connect(self.change_graph)
        self._start_normalise_mcu()


    def change_graph(self):
        """Sets up the plotting environment based on UI selection."""
        # -----------------------  Reset Plotting Area -----------------------
        self.graphicsView.clear()
        if hasattr(self, 'timer'):
            self.timer.stop()

        # ----------------------- Extract and Parse Inputs -----------------------
        mode = self.select_mode_comboBox.currentText()
        interval_str = self.timeInterval_comboBox.currentText()
        interval_ms = int(interval_str[:-2])
        
        #  ----------------------- Handle Scaling Logic -----------------------
        # factor maps 100ms -> 1, 500ms -> 5, 1000ms -> 10
        factor = interval_ms // 100
        serial_backend.TOT_COUNT_ACCUMULATE_RECV_IN_1_SEC = factor * serial_backend.TOT_COUNT_ACCUMULATE_RECV_IN_1_SEC_FRONTEND
        
        #  ----------------------- Mode-specific time axis length  -----------------------
        range_len = 50000 if (mode == "View angle" and interval_ms == 1000) else (factor * 1000)
        self.time_axis = [i * 0.0001 for i in range(range_len)]

        # ----------------------- Mode Configuration Map -----------------------
        # Structure: { ModeName: (UpdateFunction, [Plot1_Setup, Plot2_Setup]) }
        config = {
            "View sensors": (self.graph_update_sensors, [
                {"title": "Hall sensors", "y": "Hall Voltage", "u": "[norm]", "curves": [("r", "v1", "Hall 1"), ("b", "v2", "Hall 2")]},
                {"title": "Current sensors", "y": "Current", "u": "mA", "curves": [("g", "i1", "I1"), ("y", "i2", "I2")]}
            ]),
            "View angle": (self.graph_update_angle, [
                {"title": "Permanent magnet angle", "y": "ϕ_m", "u": "rad", "curves": [("r", "sigma_m", "Sigma")]},
                {"title": "Magnetic field angle", "y": "ϕ_B", "u": "rad", "curves": [("g", "sigma_b", "B")]}
            ]),
            "View phase difference": (self.graph_phase_difference, [
                {"title": "Phase Difference", "y": "Δϕ", "u": "rad", "curves": [("g", "phase_diff", "Delta")]}
            ])
        }

        if mode not in config:
            return

        update_func, plots_setup = config[mode]

        # ----------------------- Build the UI -----------------------
        for row, setup in enumerate(plots_setup):
            plot = self.graphicsView.addPlot(row=row, col=0, title=setup["title"])
            self._apply_plot_style(plot, setup["y"], setup["u"])
            
            for color, attr_suffix, name in setup["curves"]:
                curve = plot.plot(pen=color, name=name)
                # Dynamically set attribute (e.g., self.curve_v1)
                setattr(self, f"curve_{attr_suffix}", curve)

        # ----------------------- Initialize Timer -----------------------
        self.timer = QtCore.QTimer()
        self.timer.setInterval(interval_ms)
        self.timer.timeout.connect(update_func)
        self.timer.start()

    def _apply_plot_style(self, plot, y_label, units):
        """Standardizes plot appearance."""
        plot.setLabel('left', y_label, units=units)
        plot.setLabel('bottom', 'Time', units='s')
        plot.addLegend()
        plot.showGrid(x=True, y=True)
        plot.enableAutoRange(axis='x', enable=False)
        plot.setXRange(0, 0.5)

    def start_normalise_event(self):
        if not self.worker_DataUpdate.flag_normalise:
            self.worker_DataUpdate.flag_normalise_event(True)
        else:
            self.worker_DataUpdate.flag_normalise_event(False)

    def graph_update_sensors(self):
    # ----------------------- Access the slices once to minimize race condition window -----------------------
        v1 = self.worker_getter_graph.v1_slice
        v2 = self.worker_getter_graph.v2_slice
        i1 = self.worker_getter_graph.i1_slice
        i2 = self.worker_getter_graph.i2_slice

        # ----------------------- Determine the shortest length available right now -----------------------
        # We compare the time_axis and all 4 data arrays
        min_len = min(len(self.time_axis), len(v1), len(v2), len(i1), len(i2))

        # 3. Validation
        if min_len == 0:
            return

        # 4. Slice all arrays to the same length for this frame
        t = self.time_axis[:min_len]
        
        self.curve_v1.setData(t, v1[:min_len])
        self.curve_v2.setData(t, v2[:min_len])
        self.curve_i1.setData(t, i1[:min_len])
        self.curve_i2.setData(t, i2[:min_len])
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
        
        self.curve_phase_diff.setData(self.time_axis, self.worker_getter_graph.phase_difference_val)
        
        data_mutex.unlock()           
        
    def _start_normalise_mcu(self) -> None:
            self.worker_flag_send.flag_init_tx = True 

        
    def send_offsets(self):
        """
        Send offsets for earth field compensation
        """
        ############################# send data to setter getter ######################################
        self.worker_data_block.data_1 = 65534
        #make the running frequency 200 Hz, does not matter since we will produce DC current anyway
        #change to frequency for MCU
        self.worker_data_block.data_2_for_MCU  = 200 #Hz
        
        self.worker_data_block.data_current = self.textbox_offset_1.text(), 0, self.textbox_offset_2.text(), 0

        #from combobox direction
        self.worker_data_block.data_7 = 1
        self.worker_data_block.data_8 = serial_backend.CSR_COIL_CALIBRATION
        #for data 10
        self.worker_data_block.data_10 = serial_backend.CONTROL_SHEAR_RATE
        ######################################################################################
        

        ##send all data to microcontroller
        #activate flag
        self.worker_flag_send.flag_tx = True

        self.status_label.setStyleSheet("color: #32a83a;")
        self.status_label.setText("Offsets sent!")


    def send_start_vector(self):
        """
        Send start vector for magnet
        
        This function just sends a DC voltage to the microcontroller. That's all
        """
        
        ############################# send data to setter getter ######################################
        self.worker_data_block.data_1 = 65534
        #make the running frequency 200 Hz, does not matter since we will produce DC current anyway
        #change to frequency for MCU
        self.worker_data_block.data_2_for_MCU  = 200 #Hz
        
        self.worker_data_block.data_current = float(self.textbox_direction_start_x.text()) + float(self.textbox_offset_1.text()), 0, float(self.textbox_direction_start_y.text()) + float(self.textbox_offset_2.text()), 0
        #from combobox direction
        self.worker_data_block.data_7 = 1

        self.worker_data_block.data_8 = serial_backend.CSR_COIL_CALIBRATION

        
        #for data 10
        self.worker_data_block.data_10 = serial_backend.CONTROL_SHEAR_RATE
        ######################################################################################
        
        
        ##send all data to microcontroller
        #activate flag
        self.worker_flag_send.flag_tx = True
        

        self.status_label.setStyleSheet("color: #32a83a;")
        self.status_label.setText("Offsets sent!")
        
        
        
    def start_creep_test(self):
        """
        Send start vector for magnet
        """
        #### delete previous dummy files before acquisition starts.
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
        
        """ 
        Declare all the input textbox variables
        """
        start_vector_x = int(self.textbox_direction_start_x.text())
        start_vector_y = int(self.textbox_direction_start_y.text())
        end_vector_x = int(self.textbox_direction_end_x.text())
        end_vector_y = int(self.textbox_direction_end_y.text())
        
        self.start_vector_time = float(self.textbox_vector_start_time.text())
        self.end_vector_time = float(self.textbox_vector_end_time.text())
        input_sampling_time = float(self.textbox_input_sampling_time.text())
        
        
        standard_sampling_rate = int(self.textbox_standard_sampling_rate.text())
        
        #get the total time of the time taken (x-axis on the graph)
        self.worker_remaining_time.total_time_for_file_save = self.start_vector_time + self.end_vector_time
        
        ############################# send data to setter getter ######################################
        
        
        self.worker_data_block.data_1 = self.worker_remaining_time.total_time_for_file_save
        #make the running frequency 200 Hz, does not matter since we will produce DC current anyway
        #change to frequency for MCU
        self.worker_data_block.data_2_for_MCU  = 200 #Hz
        
        self.worker_data_block.data_current = float(self.textbox_direction_start_x.text()) + float(self.textbox_offset_1.text()), 0, float(self.textbox_direction_start_y.text()) + float(self.textbox_offset_2.text()), 0

        #from combobox direction
        self.worker_data_block.data_7 = 1

        self.worker_data_block.data_8 = serial_backend.CSR_COIL_CALIBRATION
        #for data 10
        self.worker_data_block.data_10 = serial_backend.CONTROL_SHEAR_RATE
        ######################################################################################
        
        ##send all data to microcontroller
        #activate flag
        self.worker_flag_send.flag_tx = True
        
        ######################################################################################
        ######## get the desired sampling frequency from the textbox
        sampling_frequency_standard = self.textbox_standard_sampling_rate.text()
        if sampling_frequency_standard == '' or sampling_frequency_standard == 0:
            pass
        else:
                
            self.worker_downsample_property.time_increment  = 1.0/float(sampling_frequency_standard)
            self.worker_downsample_property.tot_average  = 10000 // int(sampling_frequency_standard)
            
            check1 = 5000 / int(self.worker_downsample_property.tot_average)
        
        #for specified sampling frequency 
        input_sampling_rate = int(self.textbox_input_sampling_rate.text())
        self.worker_remaining_time.input_sampling_time = float(self.textbox_input_sampling_time.text())
        
        ######################################################################################

        
        ######################################################################################
        if input_sampling_rate == '' or input_sampling_rate == 0:
            pass
        else:
            self.worker_downsample_property.time_increment_specified = 1.0/float(input_sampling_rate)
            self.worker_downsample_property.tot_average_specified = 10000 // int(input_sampling_rate)
            
            check3 = 5000 / int(self.worker_downsample_property.tot_average_specified)
        ######################################################################################
        
        ######################################################################################
        if check1.is_integer() and check3.is_integer():
            self.worker_downsample_property.current_time = 0.0
            self.worker_flag_run_time.flag_running_time = True
            #start specific 
            self.worker_downsample_property.flag_specific_downsample = True
            
            serial_backend.file_name_change_set("dummy")        #set file name from gui
            print("check is", check3)



            self.status_label.setStyleSheet("color: #7da832;")
            self.status_label.setText("Creep test starts.......")
            
            self.worker_remaining_time.time_for_resetting_time = self.worker_remaining_time.total_time_for_file_save - input_sampling_time

            #start the overall timer (combined timer)
            self.worker_sleep = SleepTimer()
            self.worker_sleep.update_time_signal.connect(self.update_time_counter_acquisition)
            self.worker_sleep.start()
            
            self.worker_vector_sleep = SleepTimerVector(self.start_vector_time)
            self.worker_vector_sleep.update_time_signal_vector.connect(self.update_time_counter_start_vector)
            self.worker_vector_sleep.start()
            
        ## Error handling when invalid sampling frequency is being input
        else:
            self.popout_window(5)
            
        ######################################################################################
    
        
        
    def update_time_counter_start_vector(self, val):
        """
        for vector timer purposes
        """
        self.lcdNumber_vector.display(val)
            
        if val not in (0, 0.1):
            self.button_offsets.setDisabled(True)
            self.button_send_start_vec.setDisabled(True)
            self.button_start_acquistion.setDisabled(True)
            pass
            
        else:
            # self.button_start.setDisabled(True)
            # self.button_stop.setDisabled(True)
            
            self.end_vector_time = float(self.textbox_vector_end_time.text())
            
            #for specified sampling frequency 
            input_sampling_rate = int(self.textbox_input_sampling_rate.text())
            
            if input_sampling_rate == '' or input_sampling_rate == 0:
                pass
            else:
                self.worker_downsample_property.time_increment_specified = 1.0/float(input_sampling_rate)
                self.worker_downsample_property.tot_average_specified = 10000 // int(input_sampling_rate)
                
                check = 5000 / int(self.worker_downsample_property.tot_average_specified)
                print("check is", check)
            """
            Send the 2nd vector 
            """
            
            ############################# send data to setter getter ###################################################
            self.worker_data_block.data_1 = self.end_vector_time
            #make the running frequency 200 Hz, does not matter since we will produce DC current anyway
            #change to frequency for MCU
            self.worker_data_block.data_2_for_MCU  = 200 #Hz
            
            self.worker_data_block.data_current = float(self.textbox_direction_end_x.text()) + float(self.textbox_offset_1.text()) , 0, float(self.textbox_direction_end_y.text()) + float(self.textbox_offset_2.text()), 0

            #from combobox direction
            self.worker_data_block.data_7 = 1
            self.worker_data_block.data_8 = serial_backend.CSR_COIL_CALIBRATION
            
            #for data 10
            self.worker_data_block.data_10 = serial_backend.CONTROL_SHEAR_RATE
            ############################################################################################################
            
            ##send all data to microcontroller
            #activate flag
            self.worker_flag_send.flag_tx = True
            
            
            
            self.status_label.setStyleSheet("color: #7da832;")
            self.status_label.setText("Creep test end vector.......")
            
            self.worker_vector_sleep = SleepTimerVector(self.end_vector_time)
            self.worker_vector_sleep.update_time_signal_vector.connect(self.update_timer_end_vector)
            self.worker_vector_sleep.start()

            
    def update_timer_end_vector(self, val):            
        """
        for vector timer purposes
        """
        self.lcdNumber_vector.display(val)
            
        if val not in (0, 0.1):
            self.button_offsets.setDisabled(True)
            self.button_send_start_vec.setDisabled(True)
            self.button_start_acquistion.setDisabled(True)
        
        else:
            self.button_offsets.setDisabled(False)
            self.button_send_start_vec.setDisabled(False)
            self.button_start_acquistion.setDisabled(False)
            
            
    def update_time_counter_acquisition(self, val):
        """
        for combined timer purposes
        """
        self.lcdNumber.display(val)
            
        if val not in (0, 0.1):
            self.button_offsets.setDisabled(True)
            self.button_send_start_vec.setDisabled(True)
            self.button_start_acquistion.setDisabled(True)
        else:
            self.button_offsets.setDisabled(False)
            self.button_send_start_vec.setDisabled(False)
            self.button_start_acquistion.setDisabled(False)


    def normalisation_magnet_event(self):
        #Reset the normalise event state 
        self.worker_DataUpdate.flag_normalise_event(False)
        
        #--------------------- Update the Data Worker ---------------------
        self.worker_data_block.data_1 = 0.5 # Placeholder
        self.worker_data_block.data_2_CSR = 10  # Forced 3Hz Frequency
        self.worker_data_block.data_current = (300, 0, 300, 0)  # Coil currents

        self.worker_data_block.data_7 = 2
        self.worker_data_block.data_8 = serial_backend.CSR_NORM
        self.worker_data_block.data_10 = serial_backend.CONTROL_SHEAR_RATE  # Rotation Mode (CRITICAL)
        self.worker_data_block.data_11 = 0.0
        self.worker_flag_send.flag_tx = True
        self.status_label.setStyleSheet("color: #7da832;")
        self.status_label.setText("Rotation starts for normalising!!!!")
        self.popout_window(6)

    def set_constant(self, get_accumulate_hall_1, get_accumulate_hall_2, get_accumulate_current_1,
                     get_accumulate_current2):
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

    def graph_stop_event(self):
        if self.stop_default_state == 1:
            
            self.graph_stop_button.setText("Resume live-graph")
            self.stop_default_state = 2
            self.timer.stop()
            
        elif self.stop_default_state == 2:
            
            self.graph_stop_button.setText("Pause live-graph")
            self.stop_default_state = 1
            self.timer.start()
            
    def auto_range_event(self):
        
        self.plot1.enableAutoRange(axis='y', enable=True)
        self.plot2.enableAutoRange(axis='y', enable=True)
        # self.plot1.enableAutoRange(axis='x', enable=True)
        # self.plot2.enableAutoRange(axis='x', enable=True)
        

            
            
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
        serial_backend.p1.terminate()
        serial_backend.p1.join()

        #stop all the threads
        self.worker_socket.stop()
        self.worker_DataUpdate.stop()
        
        #restart the python script
        os.execv(sys.executable, ['python'] + sys.argv)

    def save_button_event(self):
        # Disable immediately to prevent double clicks
        # Disable immediately to prevent double clicks
        self.save_button.setEnabled(False)

        # Show the dialog (modal → blocks this function until closed)
        filename_saving, _ = QFileDialog.getSaveFileName(
            self, "Save File", "", "CSV Files (*.csv)"
        )

        # Re-enable after dialog is closed (save or cancel)
        self.save_button.setEnabled(True)
        
        if filename_saving:
            data_read = np.loadtxt(dir_dummy_csv, delimiter=';')
            np.savetxt(filename_saving, data_read, delimiter=';')

    def popout_window(self, arg):
        
        msg = QMessageBox()
        
        text = device_state.set_popout_text(arg)
        msg.setText(text)
        
        msg.setIcon(QMessageBox.Question)
        
        msg.exec()
        
        
class TabWindowCreepTest(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MiR | Mini-Rheometer")
        
        
        # Create tab widget
        tabs = QTabWidget()

        # Add your classes as tabs
        tabs.addTab(CreepTestGUI(), "Main GUI")
        tabs.addTab( AnalyseWindow(), "Data analyse")

        # Set central widget
        self.setCentralWidget(tabs)
        
        
    def closeEvent(self, event):
        #stop all the background processes

            
        for q in (serial_backend.q_to_process, serial_backend.q_to_graph, serial_backend.q_to_csv):
            q.close()
            q.join_thread()


        #terminate the other subprocess
        serial_backend.p1.terminate()
        serial_backend.p1.join()

        #stop all the threads
        
        self.main_window = CreepTestGUI()
        self.main_window.worker_socket.stop()
        self.main_window.worker_DataUpdate.stop()
        
        #DELETE THE GODDAMN FILE 
        try:
            os.remove("dummy.csv")
        except OSError as e:
            print(f"Error deleting file: {e}")
            
        event.accept()

        
        
        
        



