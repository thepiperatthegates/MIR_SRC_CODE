import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5 import *
from PyQt5.QtCore import QThread, pyqtSignal, QMutex, QRegularExpression
from PyQt5 import uic
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QIntValidator, QRegularExpressionValidator, QDoubleValidator
import sys
import os
import time
import multiprocessing
import sys
from gui_baru import Ui_Title
from main_window_test import Ui_MainWindow



import sockets_files
from sockets_files import q_to_graph

import packet_transmission
from window_show import main_2, main_3


# GLOBAL VARIABLES
checkbox_variable = 0
time_receive_thread = 0


data_1 = 0
data_2 = 0
data_3 = 0
data_4 = 0
data_5 = 0
data_6 = 0
data_7 = 0
data_8 = 0
data_9 = 0


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

time_axis =  [i * 0.0001 for i in range(5000)]

bytes_to_process = np.array([], dtype=np.uint16)  # Empty NumPy array for incoming data

flag_done = None

data_mutex = QMutex()


#function receiving data through pipe from another thread
class DataUpdate(QThread):
    def __init__(self, main_window_ref):
        super().__init__()
        self.running = True
        self.flag_calibrate = False


        self.first_calibrate = False
        self.accumulate_hall_1 = None
        self.accumulate_hall_2 = None
        self.accumulate_current_1 = None
        self.accumulate_current_2 = None
        
        self.flag_normalise = False
        

        self.total_hall_1 = None
        self.total_hall_2 = None
        self.total_current_1 = None
        self.total_current_2 = None

        self.main_window = main_window_ref


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
                i1_slice= reshaped_data[:, 2]
                i2_slice= reshaped_data[:, 3]


                
                data_mutex.lock()

                #Hall Sensors
                store_array1= packet_transmission.change_adc_hall(v1_slice)               #convert col1 (in V)
                store_array2 = packet_transmission.change_adc_hall( v2_slice)               #convert col2 (in V)
                
                #Current
                store_array3 = packet_transmission.change_current_adc(i1_slice)               #convert col3 (in mA)
                store_array4  = packet_transmission.change_current_adc(i2_slice)               #convert col4 (in mA)
                
            
        
                #Calibrate process starts
                if self.flag_calibrate:
                    self.calculate_normalise(store_array1, store_array2)
            
                #Calibrated hall sensors
                store_array2 = packet_transmission.calibrated_hall_sensors1(store_array2, store_array3/1000)  
                store_array1 = packet_transmission.calibrated_hall_sensors2(store_array1, store_array4/1000)
                
                 #this is normalising step (still do not know whether i want to do it immidiately or not)
                if self.flag_normalise == True:
                    amplitude_voltage_1 = (np.max(store_array1) - np.min(store_array1)) / 2
                    zero_offset_voltage_1 = (np.max(store_array1) + np.min(store_array1)) / 2

                    amplitude_voltage_2 = (np.max(store_array2) - np.min(store_array2)) / 2
                    zero_offset_voltage_2 = (np.max(store_array2) + np.min(store_array2)) / 2

                    store_array1 = (store_array1 - zero_offset_voltage_1) / amplitude_voltage_1
                    store_array2 = (store_array2 - zero_offset_voltage_2) / amplitude_voltage_2
                    
                #######################################################################################################
                angle_permanent_magnet_val = np.arctan2(store_array2, store_array1)
                angle_magnetic_field_val = np.arctan2(store_array3, store_array4)
                
                angle_permanent_magnet_val = np.unwrap(angle_permanent_magnet_val)
                angle_magnetic_field_val  = np.unwrap(angle_magnetic_field_val)
                #######################################################################################################
                phase_difference_val = angle_magnetic_field_val - angle_permanent_magnet_val
                data_mutex.unlock()
                
                #Clear queue after processing
                bytes_to_process = np.array([], dtype=np.uint16)


    def calculate_normalise(self, store_array1_calibrate, store_array2_calibrate):

        self.accumulate_hall_1 = np.append(self.total_hall_1, store_array1_calibrate)
        self.accumulate_hall_2 = np.append(self.total_hall_2, store_array2_calibrate)


        self.main_window.set_constant(self.accumulate_hall_1,  
                                    self.accumulate_hall_2)
        
    def flag_calibration_event(self, value1, value2):
        self.flag_calibrate = value1
        self.first_calibrate = value2

    def flag_normalise_event(self, flag_input):
        
        self.flag_normalise = flag_input

    def stop(self):
        self.running = False

        
class SleepThread(QThread):

    update_time_signal = pyqtSignal(int)            #signal for the time counter

    def __init__(self, parent=None):
        super().__init__(parent)
        self.running = True

    def run(self):
        #TODO: implement a way to kill the thread when stop button is pressed !
        global data_1     #data for run time
        # packet_transmission.running_time_event(1)
        local_data_1 = int(data_1)       #data for run time
        while local_data_1 >=0:
            if self.running:
                time.sleep(1)
                self.update_time_signal.emit(int(local_data_1))
                local_data_1 -= 1
            else:
                break
        packet_transmission.running_time_event(0)

    def stop(self):
        self.running = False
            
    
class SocketThread(QThread):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.running = True

    def run(self):
        if self.running:
            sockets_files.thread_start()

    def stop(self):
        self.running = False
    


class MyGUI(QMainWindow, Ui_Title):

    queue_file_name = multiprocessing.Queue()

    def __init__(self):
        super().__init__()
        self.setupUi(self)
        
        self.save_button.setIcon(QtGui.QIcon("save_icon.png"))
        self.button_cal_constant.setIcon(QtGui.QIcon("calibrate.png"))

        self.setWindowTitle("Mini rheometer")
        
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
        
        
        self.accumulate_hall_1 = None
        self.accumulate_hall_2  = None
        
        self.mean_hall_1_0_A = None
        self.mean_hall_2_0_A = None
        self.mean_hall_1_400_A = None
        self.mean_hall_2_0_A = None

        #########################################################################################
        #placeholder text for textboxes################################################
        self.textbox_time.setPlaceholderText("Enter time in seconds")
        self.textbox_frequency.setPlaceholderText("Enter frequency for coil currents")
        self.textbox_amplitude1.setPlaceholderText("Enter amplitude from 0 to 500mA")
        self.textbox_offset1.setPlaceholderText("Enter offset from 0 to +-500mA")
        self.textbox_amplitude2.setPlaceholderText("Enter amplitude from 0 to 500mA")
        self.textbox_offset2.setPlaceholderText("Enter offset from 0 to +-500mA")
        #################################################################################################

        #######################################################validator############################################################################
        self.textbox_time.setValidator(QIntValidator())
        self.textbox_frequency.setValidator(QDoubleValidator())
        
        #ACCEPT ONLY INTEGER FROM 0 TO 500 (unsigned)
        self.input_validator_unsigned = QRegularExpressionValidator(QRegularExpression("^(?:[0-9]|[1-9][0-9]|[1-4][0-9]{2}|500)$"), self)
        #ACCEPT ONLY INTEGER FROM -500 to 500 (signed)
        self.input_validator_signed = QRegularExpressionValidator(QRegularExpression("^-?(?:[0-9]|[1-9][0-9]|[1-4][0-9]{2}|500)$"), self)
        self.textbox_amplitude1.setValidator(self.input_validator_unsigned)
        self.textbox_offset1.setValidator(self.input_validator_signed)
        self.textbox_amplitude2.setValidator(self.input_validator_unsigned)
        self.textbox_offset2.setValidator(self.input_validator_signed)
        #######################################################validator############################################################################
        
        
        #### connected functions for button
        self.button_send.clicked.connect(self.send_parameter_event)
        self.button_send.clicked.connect(self.popout_window)
        
        self.button_start.clicked.connect(self.start_data_event)
        self.button_start.clicked.connect(self.popout_window)
        
        self.button_stop.clicked.connect(self.stop_button_push_event)
        self.button_stop.clicked.connect(self.popout_window)
        
        #calibration button
        self.button_cal_constant.clicked.connect(self.start_calibration_event)
        self.analyse_button.clicked.connect(self.analyse_button_event)
        
        #normalise button
        self.normalise_button.clicked.connect(self.start_normalise_event)

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
            if time_interval_var_int == 500:
                sockets_files.tot_count_accumulate_recv = 1250
                time_axis = [i * 0.0001 for i in range(5000)]
            elif time_interval_var_int == 1000:
                sockets_files.tot_count_accumulate_recv = 2*1250
                time_axis = [i * 0.0001 for i in range(2*5000)]
            elif time_interval_var_int == 2000:
                sockets_files.tot_count_accumulate_recv = 4*1250
                time_axis = [i * 0.0001 for i in range(4*5000)]
            
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
            self.curve_i1 = self.plot2.plot(pen='g', name="I2")
            self.curve_i2 = self.plot2.plot(pen='y', name="I1")
            
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
            if time_interval_var_int == 500:
                sockets_files.tot_count_accumulate_recv = 1250
                time_axis = [i * 0.0001 for i in range(5000)]
            elif time_interval_var_int == 1000:
                sockets_files.tot_count_accumulate_recv = 2*1250
                time_axis = [i * 0.0001 for i in range(2*5000)]
            elif time_interval_var_int == 2000:
                sockets_files.tot_count_accumulate_recv = 4*1250
                time_axis = [i * 0.0001 for i in range(4*5000)]
                
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
            if time_interval_var_int == 500:
                sockets_files.tot_count_accumulate_recv = 1250
                time_axis = [i * 0.0001 for i in range(5000)]
            elif time_interval_var_int == 1000:
                sockets_files.tot_count_accumulate_recv = 2*1250
                time_axis = [i * 0.0001 for i in range(2*5000)]
            elif time_interval_var_int == 2000:
                sockets_files.tot_count_accumulate_recv = 4*1250
                time_axis = [i * 0.0001 for i in range(4*5000)]
                
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
        global data_1
        global data_2
        global data_3
        global data_4
        global data_5
        global data_6
        global data_7        #checkbox for direction
        global data_8
        global data_9
    
        data_1 = 65534
        data_2 =self.textbox_frequency.text()
        data_2 = packet_transmission.calculate_running_frequency(float(data_2))
        data_3 = self.textbox_amplitude1.text()
        data_4 =  self.textbox_offset1.text()
        data_5 = self.textbox_amplitude2.text()
        data_6 = self.textbox_offset2.text()  

        
        #from combobox direction
        if self.comboBox_direction.currentText() == "Clockwise":
            data_7 = 1
        elif self.comboBox_direction.currentText() == "Anti-clockwise":
            data_7 = 2
            
            
        if self.filter_checkbox.isChecked():
            data_8 = 0
        else:
            data_8 = 2
        
        
        packet_transmission.send_function(data_1, data_2, data_3, data_4, data_5, data_6, data_7, data_8, data_9)
        #send all the data to be packed
        packet_transmission.send_transmission_event(1)            #SET flag for Tx
        packet_transmission.start_flag_send_event(1)
        

        
        self.status_label.setStyleSheet("color: #32a83a;")
        self.status_label.setText("Data sent!")
    
    def start_data_event(self):
        global data_1
        global data_2
        global data_3
        global data_4
        global data_5
        global data_6
        global data_7
        global data_8
        
        data_1 = self.textbox_time.text()
        data_2 = self.textbox_frequency.text()
    
        packet_transmission.stop_button_event(0)            #goto sockets_files and stop the loop for receiving
        packet_transmission.running_time_event(1)
        
        sockets_files.file_name_change_set("dummy")        #set file name from gui
        sockets_files.current_time = 0.0

        self.status_label.setStyleSheet("color: #7da832;")
        self.status_label.setText("Acquisition starts.......")


        self.queue_file_name.put("dummy") #Send file name to another process

        #kill the previous process if it exists
        if self.p_window_data is not None:
            self.p_window_data.terminate()

        #start the process at the initialisation
        self.p_window_data = multiprocessing.Process(target=main_2, args=(self.queue_file_name,))
        self.p_window_data.start()


        self.worker_sleep = SleepThread()
        self.worker_sleep.update_time_signal.connect(self.update_time_counter_acquisition)
        self.worker_sleep.start()

    def start_calibration_event(self, input_current = 1, count_recursion = 1 ):
        global data_1
        global data_2
        global data_3
        global data_4
        global data_5
        global data_6
        global data_7
        global data_8
        global data_9
        
        
        if input_current == False:
            input_current = 1
            
        
        print(input_current)

        
        data_1 = str(1) #seconds
        data_2 = str(3) # Hz
        data_3 = str(input_current)# mA
        data_4 = str(0)
        data_5 = str(input_current)  # mA
        data_6 = str(0)  # mA 

        #from combobox direction
        if self.comboBox_direction.currentText() == "Clockwise":
            data_7 = 1
        elif self.comboBox_direction.currentText() == "Anti-clockwise":
            data_7 = 2
            
            
        data_8 = 3          #mode 3 to the board
        data_9= 0

        
        packet_transmission.send_function(30 , data_2, data_3, data_4, data_5, data_6, data_7, data_8, data_9)
        # send all the data to be packed
        packet_transmission.send_transmission_event(1)            #SET flag for Tx
        packet_transmission.start_flag_send_event(1)

        # send flag for calibration in the thread
        self.worker_DataUpdate.flag_calibration_event(True, True)

        self.status_label.setStyleSheet("color: #32a83a;")
        self.status_label.setText("Calibrating!...............")
        
        QtCore.QTimer.singleShot(1000, lambda: self.after_stabilise(count_recursion))
        

        
        
        
    def after_stabilise(self, count_recursion):
        self.worker_sleep = SleepThread()
        self.worker_sleep.update_time_signal.connect(lambda value: self.update_time_counter_calibrating(value, count_recursion))
        self.worker_sleep.start()
        
    def update_time_counter_acquisition(self, val):
        self.lcdNumber.display(val)
        
        if val != 0:
            self.button_send.setDisabled(True)
            self.button_start.setDisabled(True)
            self.button_stop.setDisabled(False)

        else:
            self.button_send.setDisabled(False)
            self.button_start.setDisabled(False)
            self.button_stop.setDisabled(True)

    def update_time_counter_calibrating(self, val, count_recursion):
        self.lcdNumber.display(val)

        if val != 0:
            self.button_send.setDisabled(True)
            self.button_start.setDisabled(True)
            self.button_stop.setDisabled(False)

        elif val== 0:  #when val is 0 and the thread is stops already
            self.button_send.setDisabled(False)
            self.button_start.setDisabled(False)
            self.button_stop.setDisabled(True)
            
            self.worker_DataUpdate.flag_calibration_event(False, False)


        
            if count_recursion == 1:
                self.mean_hall_1_0_A = np.mean(self.accumulate_hall_1[1:])
                self.mean_hall_2_0_A = np.mean(self.accumulate_hall_2[1:])
                
                self.start_calibration_event(400, 2)
            elif count_recursion == 2:
                self.mean_hall_1_400_A = np.mean(self.accumulate_hall_1[1:])
                self.mean_hall_2_400_A = np.mean(self.accumulate_hall_2[1:])
                
                k_b_1 = float ((self.mean_hall_1_400_A - self.mean_hall_1_0_A) / (0.40 - (0.001)))
                k_b_2 = float ((self.mean_hall_2_400_A - self.mean_hall_2_0_A) / (0.40 - (0.001)))
                
                packet_transmission.k_b_1 = k_b_1
                packet_transmission.k_b_2 = k_b_2
                          
                print(k_b_1)
                print(k_b_2)
                
                self.popout_window()

    def stop_button_push_event(self):
        # packet_transmission.stop_button_event(1)            #goto sockets_files and stop the loop for receiving

        # TODO:important, need something to make it a little bit more sophisticated
        packet_transmission.running_time_event(0)  # STOP RECEIVING DATA FROM BOARD

        self.worker_sleep.update_time_signal.disconnect()
        self.worker_sleep.stop()

        self.lcdNumber.display(0)
        self.status_label.setStyleSheet("color: #e30000;")
        self.status_label.setText("STOP!!!") 
    
    def popout_window(self):
        msg = QMessageBox()
        msg.setText("Successful")
        msg.setIcon(QMessageBox.Question)
        
        
        x = msg.exec_()

    def analyse_button_event(self):

        #kill the previous process if clicked again
        if self.p_analyse is not None:
            self.p_analyse.terminate()

        self.p_analyse = multiprocessing.Process(target =main_3 , args=())
        self.p_analyse.start()

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
        global data_1, data_2, data_3, data_4, data_5, data_6, data_7, data_8, data_9
        
        data_9 = 1
        packet_transmission.send_function(data_1, data_2, data_3, data_4, data_5, data_6, data_7, data_8, data_9)
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
        if filename_saving:
            if not filename_saving.lower().endswith('.csv'):
                filename_saving += '.csv'
            data_read = np.loadtxt(f"dummy.csv", delimiter=';')
            np.savetxt(filename_saving, data_read, delimiter=';')

    def set_constant(self, get_accumulate_hall_1, get_accumulate_hall_2):
        
        self.accumulate_hall_1 = get_accumulate_hall_1
        self.accumulate_hall_2 = get_accumulate_hall_2


    def start_normalise_event(self):
        
        if self.worker_DataUpdate.flag_normalise == False:
            self.worker_DataUpdate.flag_normalise_event(True)
        else:
            self.worker_DataUpdate.flag_normalise_event(False)
    

    #close event to close all the threads
    def closeEvent(self, event):

        #stop all the background processes
        if self.p_window_data is not None:
            self.p_window_data.terminate()
            self.p_window_data.join()

        #terminate the other subprocess
        sockets_files.p1.terminate()
        sockets_files.p1.join()

        #stop all the threads
        self.worker_socket.stop()
        self.worker_DataUpdate.stop()
        
        #DELETE THE GODDAMN FILE 
        try:
            os.remove("dummy.csv")
        except OSError as e:
            print(f"Error deleting file: {e}")


        #close main process GUI
        event.accept()



class MainGUI(QMainWindow, Ui_MainWindow):
    
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.choose_experiment_comboBox.activated.connect(self.choose_window)
        
        
        #declare the window first without showing it 
        self.window = MyGUI()
        
    def choose_window(self):
        mode = self.choose_experiment_comboBox.currentText()
        
        if mode == "Control shear rate":
                self.window.show()
                self.close()
    


def main():
    
    app_main_window = QApplication(sys.argv)
        
    app_main_window.setWindowIcon(QtGui.QIcon('fzj.png'))
    
    first_window = MainGUI()
    first_window.show()
    sys.exit(app_main_window.exec_())


if __name__ == '__main__':
    main()
    
    
