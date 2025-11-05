import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5 import *
from PyQt5.QtCore import QThread, pyqtSignal, QMutex, QRegularExpression, QObject, QTimer
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QRegularExpressionValidator, QDoubleValidator
import os
from pathlib import Path


import packet_transmission

# #fix cache problem with MATHPLOTLIB
os.environ['MPLCONFIGDIR'] = str(Path.home()) +"/.matplotlib/"
import sys

import sockets_files 
from sockets_files import q_to_graph

from creep_test_gui import Ui_CreepTestGUI
from analyse_window_creep_test import AnalyseWindow




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
    def __init__(self):
        super().__init__()
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
                
                
                #Clear queue after processing
                bytes_to_process = np.array([], dtype=np.uint16)
                data_mutex.unlock()


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
        super().__init__()
        global data_1
        self.remaining = float(data_1) # get local_data_1 from global
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
            packet_transmission.running_time_flag_setter(0)
            
class SleepTimerVector(QObject):
    
    update_time_signal_vector = pyqtSignal(float)
    
    def __init__(self, time_taken, specified_time__sampling, flag_sampling_time):
        super().__init__()
        self.remaining = time_taken
        self.specified_time__sampling = specified_time__sampling
        self.flag_sampling_time = flag_sampling_time
        
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
        elif self.remaining == self.specified_time__sampling:
            if self.flag_sampling_time:
                socket.
        else:
            self.timer.stop()

class SocketThread(QThread):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.running = True

    def run(self):
        if self.running:
            sockets_files.thread_start()

    def stop(self):
        self.running = False
        

class CreepTestGUI(QMainWindow, Ui_CreepTestGUI):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
            
            
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
        
        self.textbox_standard_sampling_rate.setText("10000")
        ################################################################################################

        ################################ SET ICON ###################################################
        # get absolute path of project root (folder containing 'src' and 'pics')
        self.project_root = os.path.dirname(os.path.abspath(__file__))
        save_icon_path = os.path.join(self.project_root, "pics", "save_icon.ico")
        
        self.save_button.setIcon(QtGui.QIcon(save_icon_path))
        ################################################################################################
        
        
        
        
        
        ################################### connected signals from widget #############################################
        
        self.button_start_acquistion.clicked.connect(self.start_creep_test)
        self.button_offsets.clicked.connect(self.send_offsets)
        self.button_send_start_vec.clicked.connect(self.send_start_vector)
        self.save_button.clicked.connect(self.save_button_event)
        self.graph_stop_button.clicked.connect(self.graph_stop_event)
        self.button_auto_range.clicked.connect(self.auto_range_event)
        ####################################################################################
        
        ############################Start backend serial lines##################################################
        
        self.worker_socket = SocketThread()
        self.worker_DataUpdate = DataUpdate()

        self.worker_socket.start()
        self.worker_DataUpdate.start()
        ################################################################################################
        
        
        
        ##### combobox for live graph mode
        self.select_mode_comboBox.activated.connect(self.change_graph)
        ##### same as above but for time interval change
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
        data_mutex.lock()
        global angle_permanent_magnet_val, angle_magnetic_field_val
        
        self.curve_sigma_m.setData(time_axis, angle_permanent_magnet_val)
        self.curve_sigma_b.setData(time_axis, angle_magnetic_field_val)
        data_mutex.unlock()
        
    def graph_phase_difference(self):
        global phase_difference_val
        
        self.curve_phase_difference.setData(time_axis, phase_difference_val)
        
        
    def send_offsets(self):
        """
        Send offsets for earth field compensation
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
        #make the running frequency 200 Hz, does not matter since we will produce DC current anyway
        data_2 = 200 #Hz
        data_3 =0
        data_4 =  self.textbox_offset_1.text()
        data_5 = 0
        data_6 = self.textbox_offset_2.text()  

        
        ##direction does not matter here 
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
        

        self.status_label.setStyleSheet("color: #32a83a;")
        self.status_label.setText("Offsets sent!")


    def send_start_vector(self):
        """
        Send start vector for magnet
        
        This function just sends a DC voltage to the microcontroller. That's all
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
        #make the running frequency 200 Hz, does not matter since we will produce DC current anyway
        data_2 = 200 #Hz
        data_3 =0
        data_4 =  self.textbox_direction_start_x.text()
        data_5 = 0
        data_6 = self.textbox_direction_start_y.text()  

        
        ##direction does not matter here 
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
        

        self.status_label.setStyleSheet("color: #32a83a;")
        self.status_label.setText("Offsets sent!")
        
        
        
    def start_creep_test(self):
        """
        Send start vector for magnet
        """
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
        
        """ 
        Declare all the input textbox variables
        """
        start_vector_x = int(self.textbox_direction_start_x.text())
        start_vector_y = int(self.textbox_direction_start_y.text())
        end_vector_x = int(self.textbox_direction_end_x.text())
        end_vector_y = int(self.textbox_direction_end_y.text())
        
        start_vector_time = float(self.textbox_vector_start_time.text())
        end_vector_time = float(self.textbox_vector_end_time.text())
        
        
        standard_sampling_rate = int(self.textbox_standard_sampling_rate.text())
        
        #get the total time of the time taken (x-axis on the graph)
        total_time_for_file_save = start_vector_time + end_vector_time
        
        
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
        
        
        
        data_1 = total_time_for_file_save #total time for file saving
        
        #make the running frequency 200 Hz, does not matter since we will produce DC current anyway
        data_2 = 200 #Hz
        data_3 =0
        data_4 =  self.textbox_direction_start_x.text()
        data_5 = 0
        data_6 = self.textbox_direction_start_y.text()  

        
        ##direction does not matter here 
        data_7 = 1
            
        if self.filter_checkbox.isChecked():
            data_8 = 0
        else:
            data_8 = 2
            
        data_10 = 1
        
        packet_transmission.send_function(data_1, data_2, data_3, data_4, data_5, data_6, data_7, data_8, data_9, data_10)
        
        ######## get the desired sampling frequency from the textbox
        sampling_frequency = self.textbox_standard_sampling_rate.text()
        if sampling_frequency == '' or sampling_frequency == 0:
            pass
        else:
                
            sockets_files.time_increment = 1.0/float(sampling_frequency)
            sockets_files.tot_average = 10000 // int(sampling_frequency)
            print("tot_average", sockets_files.tot_average)
            
            check = 5000 / int(sockets_files.tot_average)
            print("check is", check)
        
        if check.is_integer():
            packet_transmission.stop_button_event(0)            #goto sockets_files and stop the loop for receiving
            packet_transmission.running_time_flag_setter(this_running_time_flag=1)
            
            sockets_files.file_name_change_set("dummy")        #set file name from gui
            sockets_files.current_time = 0.0


            self.status_label.setStyleSheet("color: #7da832;")
            self.status_label.setText("Creep test starts.......")
            
            #start processing to write data to csv
            packet_transmission.stop_button_event(0)            #goto sockets_files and stop the loop for receiving
            packet_transmission.running_time_flag_setter(this_running_time_flag=1)
            
            #start the overall timer (combined timer)
            self.worker_sleep = SleepTimer()
            self.worker_sleep.update_time_signal.connect(self.update_time_counter_acquisition)
            self.worker_sleep.start()
            
            self.worker_vector_sleep = SleepTimerVector(start_vector_time, 0, False)
            self.worker_vector_sleep.update_time_signal_vector.connect(self.update_time_counter_start_vector)
            self.worker_vector_sleep.start()
            
        ## Error handling when invalid sampling frequency is being input
        else:
            self.popout_window(5)
    
        
        
    def update_time_counter_start_vector(self, val):
        """
        for vector timer purposes
        """
        self.lcdNumber_vector.display(val)
            
        if val not in (0, 0.1):
            # self.button_start.setDisabled(True)
            # self.button_stop.setDisabled(True)
            print("oi")
        
        else:
            # self.button_start.setDisabled(True)
            # self.button_stop.setDisabled(True)
            
            end_vector_time = float(self.textbox_vector_end_time.text())
            input_sampling_rate = int(self.textbox_input_sampling_rate.text())
            input_sampling_time = float(self.textbox_input_sampling_time.text())
        
        
            """
            Send the 2nd vector 
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
            
        
            #make the running frequency 200 Hz, does not matter since we will produce DC current anyway
            data_2 = 200 #Hz
            data_3 = 0
            data_4 = self.textbox_direction_end_x.text()
            data_5 = 0
            data_6 = self.textbox_direction_end_y.text()  
            
            ##direction does not matter here 
            data_7 = 1
                
            if self.filter_checkbox.isChecked():
                data_8 = 0
            else:
                data_8 = 2
                
            data_10 = 1
            
            packet_transmission.send_function(data_1, data_2, data_3, data_4, data_5, data_6, data_7, data_8, data_9, data_10)
            
            packet_transmission.send_transmission_event(1)            #SET flag for Tx
            packet_transmission.start_flag_send_event(1)
            
            self.worker_vector_sleep = SleepTimerVector(end_vector_time, input_sampling_time, True)
            self.worker_vector_sleep.update_time_signal_vector.connect(self.update_timer_end_vector)
            self.worker_vector_sleep.start()
            
            self.status_label.setStyleSheet("color: #7da832;")
            self.status_label.setText("Creep test end vector.......")
            
            
            
    def update_timer_end_vector(self, val):            
        """
        for vector timer purposes
        """
        self.lcdNumber_vector.display(val)
            
        if val not in (0, 0.1):
            # self.button_start.setDisabled(True)
            # self.button_stop.setDisabled(True)
            print("oi")
        
        else:
            # self.button_start.setDisabled(True)
            # self.button_stop.setDisabled(True)
            print("oi")
            
            
    def update_time_counter_acquisition(self, val):
        """
        for combined timer purposes
        """
        self.lcdNumber.display(val)
            
        if val in (0, 0.1):
            # self.button_start.setDisabled(False)
            # self.button_stop.setDisabled(False)
            print("oi")
        else:
            # self.button_start.setDisabled(True)
            # self.button_stop.setDisabled(True)
            print("oi")
            
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

            
        for q in (sockets_files.q_to_process, sockets_files.q_to_graph, sockets_files.q_to_csv):
            q.close()
            q.join_thread()


        #terminate the other subprocess
        sockets_files.p1.terminate()
        sockets_files.p1.join()

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

        
        
        
        



