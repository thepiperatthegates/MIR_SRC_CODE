# GLOBAL VARIABLES
import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5 import *
from PyQt5.QtCore import QThread, pyqtSignal, QMutex, QRegularExpression, QObject, QTimer
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QRegularExpressionValidator, QDoubleValidator
import sys
import os
from pathlib import Path


import socket_GUI.device_state as device_state
import socket_GUI.serial_backend as serial_backend
from socket_GUI.serial_backend import q_to_graph

data_mutex = QMutex()



# function receiving data through pipe from another thread
class DataUpdate(QThread):
    """
    Thread class responsible for receiving, processing, and managing ADC data streams for Hall sensors and current sensors in real-time.

    This class reads incoming data packets from a queue, converts raw ADC values into meaningful voltage and current signals, 
    applies calibration and normalization, and computes derived quantities such as magnetic field angles and phase differences.

    The processed data is stored in a separate worker class (`StoreArrayGraph`) for visualization or further analysis.
    """

    def __init__(self, main_window_ref=None):
        super().__init__()
        self.main_window_ref = main_window_ref
        self.running = True

        self.flag_calibrate = False
        self.flag_fR_measurement = False
        self.flag_normalise = False
        self.flag_normalise_measurement = False

        self.accumulate_hall_1 = 0.0
        self.accumulate_hall_2 = 0.0
        self.accumulate_current_1 = 0.0
        self.accumulate_current_2 = 0.0

        self.total_hall_1 = None
        self.total_hall_2 = None
        self.total_current_1 = None
        self.total_current_2 = None

        self.v1_slice = np.array([], dtype=np.uint16)
        self.v2_slice = np.array([], dtype=np.uint16)
        self.i1_slice = np.array([], dtype=np.uint16)
        self.i2_slice = np.array([], dtype=np.uint16)
        self.phase_diff_slice = np.array([], dtype=np.float32)
        self.actual_torque = np.array([], dtype=np.float32)
        self.bytes_to_process = np.array([], dtype=np.uint16)

        self.angle_permanent_magnet_val = np.array([], dtype=np.float32)
        self.angle_magnetic_field_val = np.array([], dtype=np.float32)
        self.phase_difference_val = np.array([], dtype=np.float32)

        self.worker_normalise_properties = device_state.VoltageNormaliseCoefficient()
        self.worker_array_setter = device_state.StoreArrayGraph()
        self.worker_kb_property = device_state.kbCoefficient()
        self.inverted_thousand = 1.0 / 1000.0

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
        """

        data_from_pipe = []  # creating a list here because data from pipe is a list
        
        num_columns = serial_backend.num_columns  # v1, v2, i1, i2, phase_diff
        
        while self.running:
            data_from_pipe = q_to_graph.get()
            if not self.running:
                break
            if data_from_pipe:
                self.bytes_to_process = np.array(data_from_pipe, dtype=float)

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
                self.torque_slice = reshaped_data[:, 5]         # phase diff [rad]

                data_mutex.lock()
                # Current
                self.i1_slice = -device_state.change_current_adc(self.i1_slice)  # convert col3 (in mA)
                self.i2_slice = device_state.change_current_adc(self.i2_slice)  # convert col4 (in mA)

                # Calibration for current sensor
                self.i1_slice = device_state.calibration_input_coil_1(self.i1_slice)
                self.i2_slice = device_state.calibration_input_coil_2(self.i2_slice)

                #----- calibration for  hall sensors -----
                #TODO: Calibration has to be done in MCU too.
                # self.v1_slice = device_state.calibrated_hall_sensors1(self.worker_kb_property.k_b_norm_1,
                #                                                             self.v1_slice, 
                #                                                             self.i1_slice * self.inverted_thousand
                #                                                             )
                # self.v2_slice = device_state.calibrated_hall_sensors2(self.worker_kb_property.k_b_norm_2,
                #                                                             self.v2_slice, 
                #                                                             self.i2_slice * self.inverted_thousand
                #                                                             )
                
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
                self.actual_torque =  self.torque_slice

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
