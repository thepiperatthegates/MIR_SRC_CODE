
import numpy as np
from PySide6 import QtCore, QtGui
from PySide6 import *
from PySide6.QtCore import QThread, Signal, QMutex, QObject, QTimer
from PySide6.QtWidgets import *
import sys
import os
from pathlib import Path


# #fix cache problem with MATHPLOTLIB
os.environ['MPLCONFIGDIR'] = str(Path.home()) +"/.matplotlib/"
import multiprocessing
import sys
from .mapheus_main_gui import Ui_Title

import serial_comm.serial_backend as serial_backend
from serial_comm.serial_backend import q_to_graph

from serial_comm import device_state 



# GLOBAL VARIABLES
data_mutex = QMutex()


# function receiving data through pipe from another thread
class DataUpdate(QThread):
    """Reads ADC packets from the queue, converts to voltages/currents, computes angles and phase difference, and stores results for plotting."""

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
        self.v1_slice = np.array([], dtype=np.float16)
        self.v2_slice = np.array([], dtype=np.float16)
        self.i1_slice = np.array([], dtype=np.float16)
        self.i2_slice = np.array([], dtype=np.float16)
        self.bytes_to_process = np.array([], dtype=np.float32)

        #------------ Calculated Values ------------------------
        self.angle_permanent_magnet_val = np.array([], dtype=np.float32)
        self.angle_magnetic_field_val = np.array([], dtype=np.float32)
        self.phase_difference_val = np.array([], dtype=np.float32)

        # ------ Helper Workers --------------------------------
        self.worker_normalise_properties = device_state.VoltageNormaliseCoefficient()
        self.worker_array_setter = device_state.StoreArrayGraph()
        self.worker_kb_property = device_state.kbCoefficient()
        
        
    def run(self):

        """Main loop: dequeues raw ADC packets [i1, i2, v1, v2], converts, normalises, computes angles, and pushes to worker_array_setter."""
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

                # Hall Sensors
                #TODO: NEW FIRMWARE ITERACTIONS MUST NOT BE NEGATIVE FOR I1!!!!!!!!!
                # self.v1_slice = device_state.change_adc_hall(self.v1_slice)  # convert col1 (in V)
                # self.v2_slice = device_state.change_adc_hall(self.v2_slice)  # convert col2 (in V)

                # # Current
                # self.i1_slice = device_state.change_current_adc(self.i1_slice)  # convert col3 (in mA)
                # self.i2_slice = device_state.change_current_adc(self.i2_slice)  # convert col4 (in mA)

                # # calibration for current sensor
                # self.i1_slice = device_state.calibration_input_coil_1(self.i1_slice)
                # self.i2_slice = device_state.calibration_input_coil_2(self.i2_slice)


                # # calibration for  hall sensors
                # self.v1_slice = device_state.calibrated_hall_sensors1(self.worker_kb_property.k_b_1,
                #                                                              self.v1_slice, self.i1_slice / 1000)
                # self.v2_slice = device_state.calibrated_hall_sensors2(self.worker_kb_property.k_b_2,
                #                                                              self.v2_slice, self.i2_slice / 1000)
                # if self.flag_normalise:
                #     self.v1_slice = (self.v1_slice - self.worker_normalise_properties.zero_offset_voltage_1) / self.worker_normalise_properties.amp_voltage_1
                #     self.v2_slice = (self.v2_slice - self.worker_normalise_properties.zero_offset_voltage_2) / self.worker_normalise_properties.amp_voltage_2
                
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
    """Countdown timer that ticks every 100 ms and emits remaining seconds via update_time_signal until it reaches zero."""
    update_time_signal = Signal(float)  # emit float countdown values

    def __init__(self):
        super().__init__()
        self.worker_remaining = device_state.TxData()
        self.remaining = float(self.worker_remaining.data_1) # get local_data_1 from global
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
            device_state.running_time_event.clear()

class SocketThread(QThread):
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.running = True

    def run(self):
        if self.running:
            serial_backend.thread_start()


    def stop(self):
        self.running = False
    


class MAPHEUS_GUI(QMainWindow, Ui_Title):

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
        self._connect_signals()

        # --------  Start Background Services. ------------
        self._start_services()

    def _init_system_settings(self):
        """Handle DPI and Pathing."""
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
        """Initialize device_state workers."""
        self.worker_data_block = device_state.TxData()
        self.worker_getter_graph = device_state.StoreArrayGraph()
        self.worker_fr_property = device_state.fRCoefficients()
        self.worker_k_b_property = device_state.kbCoefficient()
        self.worker_downsample_property = device_state.DownSampleSpecificFlag()
        self.worker_normalise_properties = device_state.VoltageNormaliseCoefficient()
        
        #--------- Constants ---------
        self.tot_average = self.worker_downsample_property.tot_average
        self.time_increment = self.worker_downsample_property.current_time
        self.COIL_CONSTANT = device_state.COIL_CONSTANT
        self.DIPOLE_MOMENT = device_state.DIPOLE_MOMENT
        
        #--------- Global sync ---------
        device_state.CALIBRATION_FACTOR = self.worker_fr_property.CALIBRATION_FACTOR

    def _setup_ui_elements(self):
        """Set icons, placeholders, and labels."""
        #--------- Icons. ------------------
        get_path = lambda x: os.path.join(self.project_root, "pics", x)
        self.save_button.setIcon(QtGui.QIcon(get_path("save_icon.ico")))

        #--------- Text/Labels  ---------
        self.k_b_label.setText(
            f"k<sub>b1</sub> = {self.worker_k_b_property.k_b_1}&nbsp;&nbsp;&nbsp;"
            f"k<sub>b2</sub> = {self.worker_k_b_property.k_b_2}&nbsp;&nbsp;&nbsp;"
            f"K = {device_state.CALIBRATION_FACTOR}&nbsp;&nbsp;&nbsp;"
            f"f<sub>R</sub> equation = {self.worker_fr_property.fr1}x + {self.worker_fr_property.fr0}"
        )

    def _connect_signals(self):
        """Connect all UI signals."""
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
        serial_backend.TOT_COUNT_ACCUMULATE_RECV_IN_1_SEC = factor * serial_backend.TOT_COUNT_ACCUMULATE_RECV_IN_1_SEC_FRONTEND
        
        #  ----------------------- Mode-specific time axis length  -----------------------
        range_len = 50000 if (mode == "View angle" and interval_ms == 1000) else (factor * 1000)
        self.time_axis = [i * serial_backend.SAMPLE_PERIOD for i in range(range_len)]

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
        
        
    def update_time_counter_acquisition(self, val):
        self.lcdNumber.display(val)
            
        #---------  Determine if the process is "Active" (True if val is not 0 or 0.1) --------- 
        is_active = val not in (0, 0.1)
        
        #---------  Disable buttons during activity, enable them when finished ------------------ 
        self.button_start.setDisabled(is_active)
        self.save_button.setDisabled(is_active)

    def set_constant(self, get_accumulate_hall_1, get_accumulate_hall_2, get_accumulate_current_1, get_accumulate_current_2):
        """Store accumulated Hall/current sensor arrays; called by both calibration and fR measurement callbacks."""
        
        
        self.accumulate_hall_1 = get_accumulate_hall_1
        self.accumulate_hall_2 = get_accumulate_hall_2
        self.accumulate_current_1 = get_accumulate_current_1
        self.accumulate_current_2 = get_accumulate_current_2
    
    
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

        # for data 10
        self.worker_data_block.data_10 = 1
        ######################################################################################

        ##send all data to microcontroller
        #activate flag
        device_state.tx_event.set()

        self.status_label.setStyleSheet("color: #7da832;")
        self.status_label.setText("Rotation starts for normalising!!!!")

        self.worker_sleep = SleepTimer()
        self.worker_sleep.update_time_signal.connect(self.update_timer_rotation)
        self.worker_sleep.start()


        #active the flag on DataUpdate side for finding normalising parameters for the voltages
        self.worker_DataUpdate.flag_special_event(False, False, True)

    def update_timer_rotation(self, val):
        """Timer callback for the normalisation rotation: updates LCD and computes/saves normalisation coefficients when done."""
        self.lcdNumber.display(val)

        if val != 0.0:
            self.button_send.setDisabled(True)
            self.button_start.setDisabled(True)
            self.button_rotate.setDisabled(True)

        elif val == 0.0:
            self.button_send.setDisabled(False)
            self.button_start.setDisabled(False)
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
            
            device_state.VoltageNormaliseCoefficient.reload()

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
        serial_backend.p1.terminate()
        serial_backend.p1.join()

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
        
        text = device_state.set_popout_text(arg, calculate_final_fR, k_b_1, k_b_2)
        msg.setText(text)
        
        msg.setIcon(QMessageBox.Icon.Question)
        
        msg.exec()
        




#different windows with tab 
class TabWindowMAPHEUS(QMainWindow):
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MiR | Mini-Rheometer")
        self.resize(1600, 980)  # width, height
        

        # Create tab widget
        tabs = QTabWidget()

        # Add your classes as tabs
        tabs.addTab(MAPHEUS_GUI(), "Main GUI")

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
        
        self.main_window = MAPHEUS_GUI()
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
    
    csr_window = MAPHEUS_GUI()    
    csr_window.showMaximized()