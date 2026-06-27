
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
from .gui_baru import Ui_Title

import serial_comm.serial_backend as sockets_files
from serial_comm.serial_backend import q_to_graph

from serial_comm import device_state as packet_transmission
from .analyse_window_const_sr import AnalyseWindow
from .graph_fr import PlotWindow



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

        # ------ Helper Workers --------------------------------
        self.worker_normalise_properties = packet_transmission.VoltageNormaliseCoefficient()
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
        num_columns = 4

        data_from_pipe = []  # creating a list here because data from pipe is a list
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

                self.i1_slice = reshaped_data[:, 0]  # STIMMT
                self.i2_slice = reshaped_data[:, 1]  # STIMMT
                self.v1_slice = reshaped_data[:, 2]
                self.v2_slice = reshaped_data[:, 3]

                data_mutex.lock()
                print(self.i1_slice)
                # Hall Sensors
                #TODO: NEW FIRMWARE ITERACTIONS MUST NOT BE NEGATIVE FOR I1!!!!!!!!!
                self.v1_slice = packet_transmission.change_adc_hall(self.v1_slice)  # convert col1 (in V)
                self.v2_slice = packet_transmission.change_adc_hall(self.v2_slice)  # convert col2 (in V)

                # Current
                self.i1_slice = packet_transmission.change_current_adc(self.i1_slice)  # convert col3 (in mA)
                self.i2_slice = packet_transmission.change_current_adc(self.i2_slice)  # convert col4 (in mA)

                # # calibration for current sensor
                # self.i1_slice = packet_transmission.calibration_input_coil_1(self.i1_slice)
                # self.i2_slice = packet_transmission.calibration_input_coil_2(self.i2_slice)


                # # calibration for  hall sensors
                # self.v1_slice = packet_transmission.calibrated_hall_sensors1(self.worker_kb_property.k_b_1,
                #                                                              self.v1_slice, self.i1_slice / 1000)
                # self.v2_slice = packet_transmission.calibrated_hall_sensors2(self.worker_kb_property.k_b_2,
                #                                                              self.v2_slice, self.i2_slice / 1000)
                if self.flag_normalise:
                    self.v1_slice = (self.v1_slice - self.worker_normalise_properties.zero_offset_voltage_1) / self.worker_normalise_properties.amp_voltage_1
                    self.v2_slice = (self.v2_slice - self.worker_normalise_properties.zero_offset_voltage_2) / self.worker_normalise_properties.amp_voltage_2
                
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
                self.phase_difference_val = self.angle_magnetic_field_val - self.angle_permanent_magnet_val

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

    def __init__(self):
        super().__init__()
        self.setupUi(self)
        
        # -------- System & Paths --------------------
        self._init_system_settings()
        
        # -------- State & Data Variables --------------
        self._init_state_variables()
        self._init_measurement_variables()
        
        # --------  Hardware/Worker Interfaces.  -------------
        self._init_workers()
        
        # --------  UI Polish & Logic ---------------
        self._setup_ui_elements()
        self._setup_validators()
        self._connect_signals()

        # --------  Start Background Services. ------------
        self._start_services()

    def _init_system_settings(self):
        """Handle DPI and Pathing."""
        if hasattr(QtCore.Qt, 'AA_EnableHighDpiScaling'):
            QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
        if hasattr(QtCore.Qt, 'AA_UseHighDpiPixmaps'):
            QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)
            
        self.project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    def _init_state_variables(self):
        """Initialize flags and process references."""
        self.p_window_data = None
        self.p_analyse = None
        self.worker_sleep = None
        self.stop_default_state = 1
        self.flag_fR = False
        self.flag_K = False
        
        # Plot references
        self.time_axis = [i * 0.0001 for i in range(1000)]
        self._plot_ref1 = self._plot_ref2 = None
        self.before1 = self.before2 = None
        self.curve_v1 = self.curve_v2 = self.curve_i1 = self.curve_i2 = None
        self.curve_sigma_b = self.curve_sigma_m = None

    def _init_measurement_variables(self):
        """Initialize all calculation variables (Preserving original names)."""
        # Accumulators
        self.accumulate_hall_1 = self.accumulate_hall_2 = None
        self.accumulate_current_1 = self.accumulate_current_2 = None
        
        # Calibration
        self.mean_hall_1_0_A = self.mean_hall_2_0_A = None
        self.mean_hall_1_400_A = self.mean_hall_2_400_A = None # Fixed typo from original
        
        # fR Measurement arrays
        self.mean_current1_fR = self.mean_current2_fR = None
        self.mean_hall1_fR = self.mean_hall2_fR = None
        
        self.calculated_torque = np.zeros(20)
        self.calculated_angular_velocity = np.zeros(20)
        self.mean_phase = np.zeros(20)
        self.standard_mean_phase = np.zeros(20)
        self.standard_mean_torque = np.zeros(20)
        self.standard_torque = np.zeros(20)
        self.standard_phase = np.zeros(20)
        self.calculate_final_fR = 0

    def _init_workers(self):
        """Initialize packet_transmission workers."""
        self.worker_flag_run_time = packet_transmission.RunningTimeFlag()
        self.worker_data_block = packet_transmission.TxData()
        self.worker_flag_send = packet_transmission.TxFlag()
        self.worker_getter_graph = packet_transmission.StoreArrayGraph()
        self.worker_fr_property = packet_transmission.fRCoefficients()
        self.worker_k_b_property = packet_transmission.kbCoefficient()
        self.worker_downsample_property = packet_transmission.DownSampleSpecificFlag()
        self.worker_normalise_properties = packet_transmission.VoltageNormaliseCoefficient()
        
        #--------- Constants ---------
        self.tot_average = self.worker_downsample_property.tot_average
        self.time_increment = self.worker_downsample_property.current_time
        self.COIL_CONSTANT = packet_transmission.COIL_CONSTANT
        self.DIPOLE_MOMENT = packet_transmission.DIPOLE_MOMENT
        
        #--------- Global sync ---------
        packet_transmission.CALIBRATION_FACTOR = self.worker_fr_property.CALIBRATION_FACTOR

    def _setup_ui_elements(self):
        """Set icons, placeholders, and labels."""
        #--------- Icons. ------------------
        get_path = lambda x: os.path.join(self.project_root, "pics", x)
        self.save_button.setIcon(QtGui.QIcon(get_path("save_icon.ico")))
        self.button_fr_constant.setIcon(QtGui.QIcon(get_path("friction_icon.png")))
        self.button_cal_constant.setIcon(QtGui.QIcon(get_path("calibrate.png")))

        #--------- Text/Labels  ---------
        self.button_stop.setDisabled(True)
        self.textbox_time.setPlaceholderText("Enter time in second")
        self.textbox_sample_frequency.setText("10000")
        
        self.k_b_label.setText(
            f"k<sub>b1</sub> = {self.worker_k_b_property.k_b_1}&nbsp;&nbsp;&nbsp;"
            f"k<sub>b2</sub> = {self.worker_k_b_property.k_b_2}&nbsp;&nbsp;&nbsp;"
            f"K = {packet_transmission.CALIBRATION_FACTOR}&nbsp;&nbsp;&nbsp;"
            f"f<sub>R</sub> equation = {self.worker_fr_property.fr1}x + {self.worker_fr_property.fr0}"
        )
        
        self.plot_object = PlotWindow()

    def _setup_validators(self):
        """Input validation logic."""
        v_time = QDoubleValidator(0.0, 1000.0, 1)
        v_time.setNotation(QDoubleValidator.StandardNotation)
        self.textbox_time.setValidator(v_time)
        
        #--------- Regex Validators --------- 
        u_reg = QRegularExpression(r"^(?:[0-9](\.[0-9]+)?|[1-9][0-9](\.[0-9]+)?|[1-3][0-9]{2}(\.[0-9]+)?|4[0-7][0-9](\.[0-9]+)?|480(\.0+)?)$")
        s_reg = QRegularExpression(r"^-?(?:[0-9]|[1-9][0-9]|[1-3][0-9]{2}|4[0-7][0-9]|480)$")
        
        self.textbox_amplitude1.setValidator(QRegularExpressionValidator(u_reg, self))
        self.textbox_offset1.setValidator(QRegularExpressionValidator(s_reg, self))
        self.textbox_amplitude2.setValidator(QRegularExpressionValidator(u_reg, self))
        self.textbox_offset2.setValidator(QRegularExpressionValidator(s_reg, self))

    def _connect_signals(self):
        """Connect all UI signals."""
        self.button_send.clicked.connect(self.send_parameter_event)
        self.button_start.clicked.connect(self.start_data_event)
        self.button_stop.clicked.connect(self.stop_button_push_event)
        
        #--------- Add popouts ---------
        for btn in [self.button_send, self.button_start, self.button_stop, self.button_cal_constant]:
            btn.clicked.connect(lambda: self.popout_window(1))

        self.normalise_button.clicked.connect(self.start_normalise_event)
        self.button_cal_constant.clicked.connect(lambda: self.start_calibration_event(-400, 1))
        self.button_fr_constant.clicked.connect(lambda: self.start_friction_coeff_event_initiation(1, 1, 0))
        self.button_fr_constant.clicked.connect(lambda: self.popout_window(3))
        
        self.graph_stop_button.clicked.connect(self.graph_stop_event)
        self.actionHardware_reset.triggered.connect(self.set_hardware_reset_event)
        self.actionSoftware_restart.triggered.connect(self.set_software_reset_event)
        self.select_mode_comboBox.activated.connect(self.change_graph)
        self.timeInterval_comboBox.activated.connect(self.change_graph)
        self.save_button.clicked.connect(self.save_button_event)
        self.button_rotate.clicked.connect(self.button_rotate_event)

    def _start_services(self):
        """Launch background threads."""
        self.worker_socket = SocketThread()
        self.worker_socket.start()
        
        self.worker_DataUpdate = DataUpdate(self)
        
        normalise_filepath = os.path.join(self.project_root, "files", "normalise_voltage_constant.csv")

        # If file exists, then load values from csv
        if os.path.exists(normalise_filepath):
            self.worker_DataUpdate.flag_normalise_event(True)
        else:
            self.worker_DataUpdate.flag_normalise_event(False)
            
        self.worker_DataUpdate.start()
        
        self.change_graph()
        
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
        
        self.curve_phase_difference.setData(self.time_axis, self.worker_getter_graph.phase_difference_val)
        
        data_mutex.unlock()
        
        
    def send_parameter_event(self):
        
        """ 
        Sends magnet control parameters (amplitude, offsets, frequency, rotation, FIR) 
        to the STM32H757XI MCU backend.
        """
        
        #------ Map UI states to MCU constants using dictionaries. -------
        dir_map = {"Clockwise": 2, "Anti-clockwise": 1}
        
        # ----- Assign values to the worker data block --------
        self.worker_data_block.data_1 = 65534
        self.worker_data_block.data_2 = float(self.textbox_frequency.text())
        
        # ----- Case for ZERO frequency values (NOT ACCEPTED BY MCU) --------
        
        if self.worker_data_block.data_2 == 0.0:
            #set the frequency to be as small as possible 
            self.worker_data_block.data_2_for_MCU = 0.00429 #Hz
        
        # ------ Pack currents/offsets directly into a tuple. --------
        self.worker_data_block.data_current = (
            self.textbox_amplitude1.text(), self.textbox_offset1.text(),
            self.textbox_amplitude2.text(), self.textbox_offset2.text()
        )

        # --------- Apply direction and filter logic --------
        self.worker_data_block.data_7 = dir_map.get(self.comboBox_direction.currentText(), 1)
        self.worker_data_block.data_8 = 0 if self.filter_checkbox.isChecked() else 2
        self.worker_data_block.data_10 = 1 

        # ---------- Hardware and UI triggers. ------------
        self.button_stop.setDisabled(False)  # Enable stop button
        self.worker_flag_send.flag_tx = True  # Activate transmission flag
        
        #------------ Update Status   ----------------------
        self.status_label.setStyleSheet("color: #32a83a; font-weight: bold;")
        self.status_label.setText("Data sent!")
        
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
        self.worker_data_block.data_1 = self.textbox_time.text()
        self.worker_data_block.data_2 = self.textbox_frequency.text()
        self.worker_data_block.data_current = (
            self.textbox_amplitude1.text(), self.textbox_offset1.text(),
            self.textbox_amplitude2.text(), self.textbox_offset2.text()
        )
        self.worker_data_block.data_10 = 1 

        # --------- Setting downsample properties ---------
        self.worker_downsample_property.time_increment = 1.0 / fs
        self.worker_downsample_property.tot_average = tot_avg
        self.worker_downsample_property.current_time = 0.0
        self.worker_flag_run_time.flag_running_time = True
        
        sockets_files.file_name_change_set("dummy")

        #--------- UI and Timer Execution.  ------------------
        self.status_label.setStyleSheet("color: #7da832;")
        self.status_label.setText("Acquisition starts.......")
        
        self.worker_sleep = SleepTimer()
        self.worker_sleep.update_time_signal.connect(self.update_time_counter_acquisition)
        self.worker_sleep.start()
            
    def update_time_counter_acquisition(self, val):
        self.lcdNumber.display(val)
            
        #---------  Determine if the process is "Active" (True if val is not 0 or 0.1) --------- 
        is_active = val not in (0, 0.1)
        
        #---------  Disable buttons during activity, enable them when finished ------------------ 
        self.button_start.setDisabled(is_active)
        self.button_stop.setDisabled(is_active)
        self.save_button.setDisabled(is_active)

    def start_calibration_event(self, input_current = -400, count_recursion = 1 ):
        
        self.worker_DataUpdate.flag_normalise_event(False)
        
        print("Recursion in main func:", count_recursion)
        
        self.worker_k_b_property.k_b_1 = 0.0
        self.worker_k_b_property.k_b_2 = 0.0
        
        if input_current == False:
            input_current = -400
        
        print("Input current for calibration:", input_current)
    
        ############################# send data to setter getter ############################################################################
        self.worker_data_block.data_1 = float(1.0) #seconds
        self.worker_data_block.data_2_for_MCU  = str(3) # Hz
        
        self.worker_data_block.data_current = str(input_current), str(0), str(input_current), str(0)

        #from combobox direction
        if self.comboBox_direction.currentText() == "Clockwise":
            self.worker_data_block.data_7 = 2
        elif self.comboBox_direction.currentText() == "Anti-clockwise":
            self.worker_data_block.data_7 = 1
            

        self.worker_data_block.data_8 = 3
        self.worker_data_block.data_9 = 0
        #for data 10
        self.worker_data_block.data_10 = 1 
        ############################################################################################################################

        ##send all data to microcontroller
        #activate flag
        self.worker_flag_send.flag_tx = True


        # send flag for calibration in the thread
        self.worker_DataUpdate.flag_special_event(True, False, False)

        self.status_label.setStyleSheet("color: #32a83a;")
        self.status_label.setText("Calibrating!...............")
        
        QtCore.QTimer.singleShot(2000, lambda: self.after_stabilise_calibration(count_recursion))
        

    def after_stabilise_calibration(self, count_recursion):
        self.worker_sleep = SleepTimer()
        self.worker_sleep.update_time_signal.connect(lambda value: self.update_time_counter_calibrating(value, count_recursion))
        self.worker_sleep.start()
        
        
    def update_time_counter_calibrating(self, val, count_recursion):
        self.lcdNumber.display(val)

        if val != 0.0:
            self.button_send.setDisabled(True)
            self.button_start.setDisabled(True)
            self.button_stop.setDisabled(True)

        elif val == 0.0:  #when val is 0 and the thread is stops already
            
            #reset flags
            self.worker_DataUpdate.flag_special_event(False, False, False)

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
                
                self.worker_k_b_property.k_b_1 = float ((self.mean_hall_1_400_A - self.mean_hall_1_0_A) / ((self.mean_current_1_400_A - self.mean_current_1_0_A)/1000))
                self.worker_k_b_property.k_b_2  = float ((self.mean_hall_2_400_A - self.mean_hall_2_0_A) / ((self.mean_current_2_400_A - self.mean_current_2_0_A)/1000))

                
                #File path for saving
                header_text = "ELECTRONICS_FLAG;k_b_1;k_b_2"
                path_to_save_kb = os.path.join(os.path.join(self.project_root, "files", "k_b_coefficient.csv"))
                
                if os.path.exists(path_to_save_kb):
                    data = np.genfromtxt(path_to_save_kb, delimiter=";", names=True)
                else:
                    #create empty template file if no file exist prior
                    data = np.array([], dtype = [
                        ("ELECTRONICS_FLAGS", "i8"), 
                        ("k_b_1", "f8"), 
                        ("k_b_2", "f8")
                    ])
                    
                #---- get the current used eletronic flags ----
                
                electronic_flags = packet_transmission.get_electronics_flag()
                index = data["ELECTRONICS_FLAG"] == electronic_flags
                
                if np.any(index):
                    data["k_b_1"][index] = self.worker_k_b_property.k_b_1
                    data["k_b_2"][index] = self.worker_k_b_property.k_b_2
                else:
                    new_row = np.array(
                        [(electronic_flags, self.worker_k_b_property.k_b_1, self.worker_k_b_property.k_b_2)],
                        dtype=data.dtype
                    )
                    data = np.concatenate((data, new_row))

                #---- Save the file again ----
                np.savetxt(
                    path_to_save_kb,
                    data,
                    delimiter=";",
                    fmt=["%d", "%.17g", "%.17g"],
                    header="ELECTRONICS_FLAG;k_b_1;k_b_2",
                    comments=""
                )
                
                self.k_b_label.setText(
                    f"k<sub>b1</sub> = {self.worker_k_b_property.k_b_1}&nbsp;&nbsp;&nbsp;"
                    f"k<sub>b2</sub> = {self.worker_k_b_property.k_b_2}&nbsp;&nbsp;&nbsp;"
                    f"K = {packet_transmission.CALIBRATION_FACTOR}&nbsp;&nbsp;&nbsp;"
                    f"f<sub>R</sub> equation = {self.worker_fr_property.fr1}x + {self.worker_fr_property.fr0}"
                )
                
                #---- reload the file ----
                packet_transmission.kbCoefficient.reload()
                
                self.popout_window(4, self.calculate_final_fR, self.worker_k_b_property.k_b_1, self.worker_k_b_property.k_b_2)
                #enable the button again
                self.button_send.setDisabled(False)
                self.button_start.setDisabled(False)
                self.button_stop.setDisabled(False)
                
                
    def set_constant(self, get_accumulate_hall_1, get_accumulate_hall_2, get_accumulate_current_1, get_accumulate_current_2):
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
        self.accumulate_current_2 = get_accumulate_current_2
    
    
    #initiation!!!!!
    def start_friction_coeff_event_initiation(self, running_frequency = 1,  rotation_direction =  1, count_recursion = 0, input_current = 40):

        #reset all f_R at first 
        if self.flag_fR == False:
            self.flag_fR = True  
            self.worker_fr_property.fr0 = 0.0
            self.worker_fr_property.fr1 = 0.0
            self.calculate_final_fR = 0
            
        self.worker_DataUpdate.flag_normalise_event(False)
            
        ############################# send data to setter getter ############################################################################
        self.worker_data_block.data_1 = str(5) #mA
        self.worker_data_block.data_2_for_MCU  = str(running_frequency) # Hz
        
        self.worker_data_block.data_current = str(200), self.textbox_offset1.text(), str(200), self.textbox_offset2.text()
        self.worker_data_block.data_7 = rotation_direction
            
        if self.filter_checkbox.isChecked():
            self.worker_data_block.data_8 = 0
        else:
            self.worker_data_block.data_8 = 2
            
        self.worker_data_block.data_9 = 0
        #for data 10
        self.worker_data_block.data_10 = 1 
        ############################################################################################################################

        ##send all data to microcontroller
        #activate flag
        self.worker_flag_send.flag_tx = True
        

        self.status_label.setStyleSheet("color: #32a83a;")
        self.status_label.setText("f<sub>R</sub> begins!...............")
        

        QtCore.QTimer.singleShot(
            1000* int(self.worker_data_block.data_1),
            lambda: self.start_friction_coeff_event(input_current, running_frequency, rotation_direction, count_recursion)
        )

    def start_friction_coeff_event(self, input_current, running_frequency = 1,  rotation_direction =  1, count_recursion = 0) -> None:
        
        ############################# send data to setter getter ######################################
        self.worker_data_block.data_1 = float(10)
        self.worker_data_block.data_2_for_MCU  = str(running_frequency) # Hz

        self.worker_data_block.data_current = input_current, self.textbox_offset1.text(), input_current, self.textbox_offset2.text()
        self.worker_data_block.data_7 = rotation_direction
            
        if self.filter_checkbox.isChecked():
            self.worker_data_block.data_8 = 0
        else:
            self.worker_data_block.data_8 = 2
            
        self.worker_data_block.data_9 = 0
        #for data 10
        self.worker_data_block.data_10 = 1 
        ######################################################################################
        
        self.status_label.setText(
            f"Recursion {count_recursion} with direction {self.worker_data_block.data_7} and frequency of {self.worker_data_block.data_2}"
        )
        
        ##send all data to microcontroller
        #activate flag
        self.worker_flag_send.flag_tx = True
        
        # send flag for calibration in the thread
        self.worker_DataUpdate.flag_special_event(False, True, False)

        time_delay = 10 * 1000
        QtCore.QTimer.singleShot(time_delay, 
                                lambda: self.after_stabilise_fR_measurement(count_recursion, running_frequency, input_current, 
                                float(self.textbox_offset1.text()), float(self.textbox_offset2.text())))
    

        
    def after_stabilise_fR_measurement(self, count_recursion, running_frequency, input_current, data_4, data_6):
        self.worker_sleep = SleepTimer()
        self.worker_sleep.update_time_signal.connect(lambda value: self.update_time_counter_fR_measurement( value, count_recursion, running_frequency, input_current, data_4, data_6))
        self.worker_sleep.start()
        
    
    def update_time_counter_fR_measurement(self, val, count_recursion, running_frequency, input_current, data_4, data_6):
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

        if val != 0.0:   # means val != 0 and val != 0.1
            self.button_send.setDisabled(True)
            self.button_start.setDisabled(True)
            self.button_stop.setDisabled(True)

        elif val == 0.0:     # means val == 0 or val == 0.1     
            #reset flags (so that accumulate does not change here )(data integrity reason here)
            self.worker_DataUpdate.flag_special_event(False, False, False)
            
            
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
                    1: 50,  2: 50,  3: 60,  4: 60,  5: 70,
                    6: 70,  7:80,  8: 90,  9: 100,
                    10: 40, 11: 50, 12: 50, 13: 50, 14: 70,
                    15: 70, 16: 70, 17: 80, 18: 90, 19: 90
                }

                input_current = current_map.get(count_recursion, 0)  # default 0 if not found
                print("Input current is", input_current)

                    
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
                # Save to CSV      
                np.savetxt(dir_fr_results, data_to_save, delimiter=";", comments="", fmt="%.17g", header=header_text)
                #calculate final fR
                self.calculate_final_fR =  np.polyfit(self.calculated_angular_velocity, self.calculated_torque, 1)
                self.popout_window(4, self.calculate_final_fR, self.worker_k_b_property.k_b_1, self.worker_k_b_property.k_b_2)
                
                self.worker_fr_property.fr1, self.worker_fr_property.fr0 = self.calculate_final_fR
                
                #change textbox in the bottom of the GUI 
                self.k_b_label.setText(
                    f"k<sub>b1</sub> = {self.worker_k_b_property.k_b_1}&nbsp;&nbsp;&nbsp;"
                    f"k<sub>b2</sub> = {self.worker_k_b_property.k_b_2}&nbsp;&nbsp;&nbsp;"
                    f"K = {packet_transmission.CALIBRATION_FACTOR}&nbsp;&nbsp;&nbsp;"
                    f"f<sub>R</sub> equation = {self.worker_fr_property.fr1}x + {self.worker_fr_property.fr0}"
                )
                
                #refresh the graph
                self.plot_object.refresh_graph()
                
                #reset the flag again
                self.flag_fR = False
                
                #enable the button again
                self.button_send.setDisabled(False)
                self.button_start.setDisabled(False)
                self.button_stop.setDisabled(False)
                
                
                if val == 0:
                    #enable the button again
                    self.button_send.setDisabled(False)
                    self.button_start.setDisabled(False)
                    self.button_stop.setDisabled(False)
                
        if val == 0:
            #enable the button again
            self.button_send.setDisabled(False)
            self.button_start.setDisabled(False)
            self.button_stop.setDisabled(False)
    
    


    def stop_button_push_event(self):
        
        ########################### gives -500mA to the coil 1 and +500mA DC to completely stop the rotation of the magnet 
        
        ############################# send data to setter getter ######################################
        self.worker_data_block.data_1 = 65534
        self.worker_data_block.data_2_for_MCU  = self.textbox_frequency.text()
        
        self.worker_data_block.data_current = 0, -500, 0, 500

        #from combobox direction
        if self.comboBox_direction.currentText() == "Clockwise":
            self.worker_data_block.data_7 = 2
            self.worker_data_block.data_7 = self.worker_data_block.data_7
        elif self.comboBox_direction.currentText() == "Anti-clockwise":
            self.worker_data_block.data_7 = 1
            self.worker_data_block.data_7 = self.worker_data_block.data_7
            
        if self.filter_checkbox.isChecked():
            self.worker_data_block.data_8 = 0
        else:
            self.worker_data_block.data_8 = 2
            
        
        #for data 10
        self.worker_data_block.data_10 = 1 
        ######################################################################################

        
        #send all data to microcontroller
        #activate flag
        self.worker_flag_send.flag_tx = True
        self.status_label.setText("Stop rotation!!....")


        self.button_rotate.setDisabled(False)


    def button_rotate_event(self):
        #Reset the normalise event state 
        self.worker_DataUpdate.flag_normalise_event(False)
        

        ############################# send data to setter getter ######################################
        self.worker_data_block.data_1 =5.0 #second
        # make the running frequency 200 Hz, does not matter since we will produce DC current anyway
        # change to frequency for MCU
        self.worker_data_block.data_2_for_MCU = 2 #Hz

        self.worker_data_block.data_current = 400, 0, 400, 0

        # from combobox direction
        self.worker_data_block.data_7 = 1

        if self.filter_checkbox.isChecked():
            self.worker_data_block.data_8 = 0
        else:
            self.worker_data_block.data_8 = 2

        # for data 10
        self.worker_data_block.data_10 = 1
        ######################################################################################

        ##send all data to microcontroller
        #activate flag
        self.worker_flag_send.flag_tx = True

        self.status_label.setStyleSheet("color: #7da832;")
        self.status_label.setText("Rotation starts for normalising!!!!")

        self.worker_sleep = SleepTimer()
        self.worker_sleep.update_time_signal.connect(self.update_timer_rotation)
        self.worker_sleep.start()


        #active the flag on DataUpdate side for finding normalising parameters for the voltages
        self.worker_DataUpdate.flag_special_event(False, False, True)

    def update_timer_rotation(self, val):
        """
        for rotation timer purposes
        """
        self.lcdNumber.display(val)

        if val != 0.0:
            self.button_send.setDisabled(True)
            self.button_start.setDisabled(True)
            self.button_stop.setDisabled(True)
            self.button_rotate.setDisabled(True)
            self.normalise_button.setDisabled(True)

        elif val == 0.0:
            self.button_send.setDisabled(False)
            self.button_start.setDisabled(False)
            self.button_stop.setDisabled(False)
            self.normalise_button.setDisabled(False)
            self.save_button.setDisabled(False)


            #mark the end of the normalise measurement event
            self.worker_DataUpdate.flag_special_event(False, False, False)


            self.worker_normalise_properties.amp_voltage_1 = (np.max(self.accumulate_hall_1[10:]) - np.min(self.accumulate_hall_1[10:])) / 2
            self.worker_normalise_properties.zero_offset_voltage_1 = (np.max(self.accumulate_hall_1[10:]) + np.min(self.accumulate_hall_1[10:])) / 2

            self.worker_normalise_properties.amp_voltage_2 = (np.max(self.accumulate_hall_2[10:]) - np.min(self.accumulate_hall_2[10:])) /  2
            self.worker_normalise_properties.zero_offset_voltage_2 = (np.max(self.accumulate_hall_2[10:]) + np.min(self.accumulate_hall_2[10:])) / 2
            
            print("self.worker_normalise_properties.amp_voltage_1", self.worker_normalise_properties.amp_voltage_1)
            print("self.worker_normalise_properties.zero_offset_voltage_1", self.worker_normalise_properties.zero_offset_voltage_1)
            print("self.worker_normalise_properties.amp_voltage_2 ", self.worker_normalise_properties.amp_voltage_2 )
            print("self.worker_normalise_properties.zero_offset_voltage_2 ", self.worker_normalise_properties.zero_offset_voltage_2 )

            #File path for saving
            header_text = "voltage_amp_1_V;voltage_zero_offset_1_V;voltage_amp_2_V;voltage_zero_offset_2_V"
            dir_save_normalisation = os.path.join(self.project_root, "files", "normalise_voltage_constant.csv")
            
            data_to_save = np.column_stack((self.worker_normalise_properties.amp_voltage_1, self.worker_normalise_properties.zero_offset_voltage_1,
                                            self.worker_normalise_properties.amp_voltage_2, self.worker_normalise_properties.zero_offset_voltage_2))
            
            print(self.worker_normalise_properties.amp_voltage_1)

            # Save to CSV      
            np.savetxt(dir_save_normalisation, data_to_save, delimiter=";", comments="", fmt="%.17g",  header=header_text)
            
            packet_transmission.VoltageNormaliseCoefficient.reload()

            self.worker_DataUpdate.flag_normalise_event(True)


            self.popout_window(6)

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
            np.savetxt(filename_saving, data_read, delimiter=';', fmt='%.17g')

        
    def popout_window(self, arg, calculate_final_fR = 0.0, k_b_1 = 0.0, k_b_2 = 0.0):
        
        msg = QMessageBox()
        
        text = packet_transmission.set_popout_text(arg, calculate_final_fR, k_b_1, k_b_2)
        msg.setText(text)
        
        msg.setIcon(QMessageBox.Question)
        
        msg.exec()
        




#different windows with tab 
class TabWindowConstSR(QMainWindow):
    
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
    


if __name__ == '__main__':
    multiprocessing.freeze_support()
    
    csr_window = TabWindowConstSR()    
    csr_window.showMaximized()