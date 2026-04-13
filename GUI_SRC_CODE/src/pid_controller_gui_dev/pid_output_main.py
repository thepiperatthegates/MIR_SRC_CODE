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
from .analyse_window_pid import AnalyseWindow



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

                self.v1_slice = reshaped_data[:, 0]     # normalised hall 1 [no unit]
                self.v2_slice = reshaped_data[:, 1]     #normalised hall 2 [no unit]
                self.i1_slice = reshaped_data[:, 2]     #i1 [digital]
                self.i2_slice = reshaped_data[:, 3]     # i2 [digital]
                self.phase_diff_slice = reshaped_data[:, 4]         # phase diff [rad]


                data_mutex.lock()

                # Hall Sensors
                
                # self.v1_slice = packet_transmission.change_adc_hall(self.v1_slice)  # convert col1 (in V)
                # self.v2_slice = packet_transmission.change_adc_hall(self.v2_slice)  # convert col2 (in V)

                # Current
                self.i1_slice = -packet_transmission.change_current_adc(self.i1_slice)  # convert col3 (in mA)
                self.i2_slice = packet_transmission.change_current_adc(self.i2_slice)  # convert col4 (in mA)

                # Calibration for current sensor
                self.i1_slice = packet_transmission.calibration_input_coil_1(self.i1_slice)
                self.i2_slice = packet_transmission.calibration_input_coil_2(self.i2_slice)


                # # calibration for  hall sensors
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
        self.remaining = float(self.worker_remaining.time_acquisition) # get local_data_1 from global
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
        
        #----------------------- High DPI and Pathing -----------------------
        self._configure_display()
        self._init_paths()
        
        #----------------------- Variable Initialization -----------------------
        self._init_plot_variables()
        self._init_data_placeholders()
        self._init_workers()
        self._init_physics_constants()
        
        #----------------------- UI and Hardware Setup -----------------------
        self._setup_initial_ui_state()
        self._start_backend_threads()
        self._connect_signals()
        self._setup_ui_elements()

    def _configure_display(self):
        """Handle High DPI scaling settings."""
        if hasattr(QtCore.Qt, 'AA_EnableHighDpiScaling'):
            QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
        if hasattr(QtCore.Qt, 'AA_UseHighDpiPixmaps'):
            QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)

    def _init_paths(self):
        """Construct absolute paths and load button icons."""
        self.project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        
        #----------------------- Mapping buttons to icon filenames -----------------------
        icons = {
            self.save_button: "save_icon.ico",
            self.button_fr_constant: "friction_icon.png",
            self.button_cal_constant: "calibrate.png"
        }
        
        for button, filename in icons.items():
            path = os.path.join(self.project_root, "pics", filename)
            button.setIcon(QtGui.QIcon(path))

    def _init_plot_variables(self):
        """Initialize references for plotting and time axes."""
        self.p_window_data = self.p_analyse = self.worker_sleep = None
        self._plot_ref1 = self._plot_ref2 = self.before1 = self.before2 = None
        self.curve_v1 = self.curve_v2 = self.curve_i1 = self.curve_i2 = None
        self.curve_sigma_b = self.curve_sigma_m = None
        self.curve_phase_difference = None
        
        self.time_axis = [i * 0.0001 for i in range(1000)]
        self.flag_fR = self.flag_K = False

    def _init_data_placeholders(self):
        """Pre-allocate arrays for torque, phase, and hall sensors."""
        # ----------------------- Accumulators -----------------------
        self.accumulate_hall_1 = self.accumulate_hall_2 = None
        self.accumulate_current_1 = self.accumulate_current_2 = None
        
        # ----------------------- Calibration Means -----------------------
        self.mean_hall_1_0_A = self.mean_hall_2_0_A = None
        self.mean_hall_1_400_A = None
        
        # ----------------------- fR Measurement arrays (20 samples) -----------------------
        self.calculated_torque = np.zeros(20)
        self.calculated_angular_velocity = np.zeros(20)
        self.mean_phase = np.zeros(20)
        self.standard_mean_phase = np.zeros(20)
        self.standard_mean_torque = np.zeros(20)
        self.standard_torque = np.zeros(20)
        self.standard_phase = np.zeros(20)
        self.calculate_final_fR = 0

    def _init_workers(self):
        """Initialize all setter/getter flags from packet_transmission."""
        self.worker_flag_run_time = packet_transmission.RunningTimeFlag()
        self.worker_data_block = packet_transmission.TxData()
        self.worker_flag_send = packet_transmission.TxFlag()
        self.worker_getter_graph = packet_transmission.StoreArrayGraph()
        self.worker_fr_property = packet_transmission.fRCoefficients()
        self.worker_k_b_property = packet_transmission.kbCoefficient()
        self.worker_downsample_property = packet_transmission.DownSampleSpecificFlag()
        self.worker_normalise_properties = packet_transmission.VoltageNormaliseCoefficient()
        
        # ----------------------- Sync downsampling local vars -----------------------
        self.tot_average = self.worker_downsample_property.tot_average
        self.time_increment = self.worker_downsample_property.current_time

    def _init_physics_constants(self):
        """Load static physics constants and coefficients."""
        self.COIL_CONSTANT = packet_transmission.COIL_CONSTANT
        self.DIPOLE_MOMENT = packet_transmission.DIPOLE_MOMENT
        self.worker_get = packet_transmission.fRCoefficients()
        packet_transmission.CALIBRATION_FACTOR = self.worker_get.CALIBRATION_FACTOR

    def _setup_initial_ui_state(self):
        """Configure default window and button states."""
        self.setWindowTitle("Const Shear Rate Window")

    def _start_backend_threads(self):
        """Launch serial communication and data update threads."""
        self.worker_socket = SocketThread()
        self.worker_DataUpdate = DataUpdate(self)
        self.worker_socket.start()
        self.worker_DataUpdate.start()

    def _connect_signals(self):
        """Link UI interactions to their respective methods."""
        # ----------------------- Buttons ----------    print(len(data))-------------
        self.button_send.clicked.connect(self.send_parameter_event)
        self.button_auto_range.clicked.connect(self.auto_range_event)
        self.save_button.clicked.connect(self.save_button_event)
        self.button_rotate.clicked.connect(self.normalisation_magnet_event)
        self.button_stop.clicked.connect(self.stop_button_push_event)
        
        
        self.button_start.clicked.connect(self.start_data_event)
        # self.button_start.clicked.connect(sef.)
        
        
        for popout in [self.button_send, self.button_rotate]:
            popout.clicked.connect(lambda: self.popout_window(1))

        # ----------------------- Actions -----------------------
        self.actionHardware_reset.triggered.connect(self.set_hardware_reset_event)
        self.actionSoftware_restart.triggered.connect(self.set_software_reset_event)
        
        # ----------------------- Live Graph Controls -----------------------
        self.select_mode_comboBox.activated.connect(self.change_graph)
        self.timeInterval_comboBox.activated.connect(self.change_graph)
        
        # ----------------------- Initial graph trigger -----------------------
        self.change_graph()

    def _setup_ui_elements(self):

        self.k_b_label.setText(
            f"k<sub>b1</sub> = {self.worker_k_b_property.k_b_1}&nbsp;&nbsp;&nbsp;"
            f"k<sub>b2</sub> = {self.worker_k_b_property.k_b_2}&nbsp;&nbsp;&nbsp;"
            f"K = {packet_transmission.CALIBRATION_FACTOR}&nbsp;&nbsp;&nbsp;"
            f"f<sub>R</sub> equation = {self.worker_fr_property.fr1}x + {self.worker_fr_property.fr0}"
        )

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
        sockets_files.TOT_COUNT_ACCUMULATE_RECV_IN_1_SEC = factor * sockets_files.TOT_COUNT_ACCUMULATE_RECV_IN_1_SEC_FRONTEND
        
        #  ----------------------- Mode-specific time axis length  -----------------------
        range_len = 50000 if (mode == "View angle" and interval_ms == 1000) else (factor * 1000)
        self.time_axis = [i * 0.0001 for i in range(range_len)]

        # ----------------------- Mode Configuration Map -----------------------
        # Structure: { ModeName: (UpdateFunction, [Plot1_Setup, Plot2_Setup]) }
        config = {
            "View sensors": (self.graph_update_sensors, [
                {"title": "Hall sensors", "y": "Voltage", "u": "V", "curves": [("r", "v1", "Hall 1"), ("b", "v2", "Hall 2")]},
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

    def send_parameter_event(self):
        """
        Validates UI inputs and transmits parameters to the STM32H757XI MCU.
        Handles shear rate, direction, filtering, and shear stress.
        """
        try:
            # --------------------- Parse and Validate Inputs ---------------------
            shear_rate = float(self.textbox_shear_rate.text() or 0)
            delta_phi = float(self.textbox_phase_difference.text() or 0)
            torque_ref = float(self.textbox_input_torque.text() or 0)
            # --------------------- Map UI States to Hardware Codes ---------------------
            # Direction: Anti-clockwise = 1, Clockwise = 2
            direction_code = 2 if self.comboBox_direction.currentText() == "Clockwise" else 1

            # --------------------- Update the Data Worker ---------------------
            self.worker_data_block.data_1 = shear_rate
            self.worker_data_block.data_2 = torque_ref
            self.worker_data_block.data_7 = direction_code
            self.worker_data_block.data_8 = sockets_files.PID_LOOP
            self.worker_data_block.data_10 = sockets_files.PID_START  # Command/Mode selector (PID CONTROLLER ON)
            self.worker_data_block.data_11 = delta_phi

            #---------------------  Physical Transmission ---------------------
            # Setting the flag triggers the actual transmission thread
            self.worker_flag_send.flag_tx = True

            # UI Feedback
            self._update_transmission_ui(success=True)

        except ValueError:
            # --------------------- If the user entered invalid text in the textboxes ---------------------
            self._update_transmission_ui(success=False, message="Invalid Input! Numbers only.")

    def normalisation_magnet_event(self):
        """
        Forces the magnet into rotation mode (3Hz) with specific coil currents.
        """
        try:
            delta_phi = float(self.textbox_phase_difference.text() or 0)

            # --------------------- Map UI logic ------------------------
            direction = 2 if self.comboBox_direction.currentText() == "Clockwise" else 1

            #--------------------- Update the Data Worker ---------------------
            self.worker_data_block.data_1 = 0.5 # Placeholder
            self.worker_data_block.data_2 = 10  # Forced 3Hz Frequency
            self.worker_data_block.data_current = (300, 0, 300, 0)  # Coil currents

            self.worker_data_block.data_7 = direction
            self.worker_data_block.data_8 = sockets_files.PID_NORM
            self.worker_data_block.data_10 = sockets_files.PID_START  # Rotation Mode (CRITICAL)
            self.worker_data_block.data_11 = delta_phi

            # ---------------------  Physical Transmission ---------------------
            # Setting the flag triggers the actual transmission thread
            self.worker_flag_send.flag_tx = True
            self._update_transmission_ui(True, "Rotating!")

        except ValueError:
            # --------------------- If the user entered invalid text in the textboxes ---------------------
            self._update_transmission_ui(False, "Input Error: Check input values!")
            
    def start_data_event(self):

            #  --------- Cleanup old files  --------------
            dummy_path = os.path.join(self.project_root, "files", "dummy.csv")
            if os.path.exists(dummy_path):
                try:
                    os.remove(dummy_path)
                except OSError as e:
                    print(f"File cleanup failed: {e}")

            #  --------- Gather Inputs & Safety Check  ------------
            fs_text = self.textbox_sample_frequency.text()
            if not fs_text or int(fs_text) == 0:
                return  # Silently exit if frequency is invalid/empty

            #  --------- Downsampling Logic & Validation.  ---------
            fs = float(fs_text)
            tot_avg = 10000 // int(fs)
            
            #  --------- Validation: Ensure 5000 is divisible by tot_average  ---------
            if not (5000 / tot_avg).is_integer():
                return self.popout_window(5)

            #   --------- Assignment (Success Path).  ------------------
                        # --------- Mapping textbox data to worker block ---------
            self.worker_data_block.time_acquisition = float(self.textbox_time.text())
            
            # --------- Setting downsample properties ---------
            self.worker_downsample_property.time_increment = 1.0 / fs
            self.worker_downsample_property.tot_average = tot_avg
            self.worker_downsample_property.current_time = 0.0
            self.worker_flag_run_time.flag_running_time = True
            
            sockets_files.file_name_change_set("dummy")

            #--------- UI and Timer Execution.  ------------------
            self._update_transmission_ui(True, "Saving!..")
            
            self.worker_sleep = SleepTimer()
            self.worker_sleep.update_time_signal.connect(self.update_time_counter_acquisition)
            self.worker_sleep.start()
            
    def update_time_counter_acquisition(self, val):
            self.lcdNumber.display(val)
            
            print(val)
            
    def stop_button_push_event(self):
        
        ########################### gives -500mA to the coil 1 and +500mA DC to completely stop the rotation of the magnet 
        
        ############################# send data to setter getter ######################################
        self.worker_data_block.data_1 = 65534
        self.worker_data_block.data_2_for_MCU  = 2.0
        
        self.worker_data_block.data_current = 0, 0, 0, 0


        self.worker_data_block.data_8 = sockets_files.PID_STOP
        #for data 10
        self.worker_data_block.data_10 = sockets_files.PID_START 
        ######################################################################################

        
        #---------------------  Physical Transmission ---------------------
        # Setting the flag triggers the actual transmission thread
        self.worker_flag_send.flag_tx = True

        # UI Feedback
        self._update_transmission_ui(success=True)

    def _update_transmission_ui(self, success, message="Data sent!"):
        """Handles button states and status labels."""
        if success:
            self.status_label.setStyleSheet("color: #32a83a;")  # Green
            self.status_label.setText(message)
        else:
            self.status_label.setStyleSheet("color: #a83232;")  # Red
            self.status_label.setText(message)
            
        
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
        tabs.addTab( AnalyseWindow(), "Data analyse")
        # Set central widget
        self.setCentralWidget(tabs)
        
    def closeEvent(self, event):
        #stop all the background processes

        print("Initiating Shutdown...")

        # 1. SEND SIGNAL: Tell the process to stop
        # Do NOT call q.close() here yet!
        for q in (sockets_files.q_to_process, sockets_files.q_to_graph, 
                sockets_files.q_to_csv, sockets_files.q_to_norm):
            try:
                # Clear the queue if it's full so the 'None' can get in
                while not q.empty():
                    q.get_nowait()
                q.put(None)
            except:
                pass

        # 2. WAIT: Give the background process time to see the 'None' and exit
        p1 = getattr(sockets_files, 'p1', None)
        if p1 is not None and p1.is_alive():
            # join() blocks the GUI for up to 2 seconds waiting for p1 to finish
            p1.join(timeout=4.0) 
            
            if p1.is_alive():
                print("Process stubborn, terminating...")
                p1.terminate()
                p1.join()

        # 3. CLEANUP: Now that the process is DEAD, it's safe to close queues
        for q in (sockets_files.q_to_process, sockets_files.q_to_graph, 
                sockets_files.q_to_csv, sockets_files.q_to_norm):
            try:
                # Setting cancel_join_thread prevents the GUI from hanging 
                # if there's still unread data in the buffer
                q.cancel_join_thread() 
                q.close()
                q.terminate()
                
                
            except:
                pass

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
    
