#3rd party lazy modules

from math import sin, cos, pi
import struct
import numpy

#for data Rx and TX
data_1 = 0
data_2 = 0
data_3 = 0
data_4 = 0
data_5 = 0
data_6 = 0
data_7 = 0
data_8 = 0
data_9 = 0


#for button event flag 
flag_send = 0
running_time_flag = 0

#for run time flag
run_time_gui = 0 

#for file name flag
file_name_flag = 0

#for clean up raw data flag
clean_up_flag = 0


#stop button flag
stop_button_flag = 1

#for start event
start_flag_rx = 0
start_flag_send = 0


#b
RESB_16 = 65535
RESB_12 = 4095

#for impedance matching adc current 
MAX_V_BEFORE_CURRENT = 5.0
MIN_V_BEFORE_CURRENT = -5.0   #or y-intercept

MAX_V_AFTER_CURRENT = 3.3
MIN_V_AFTER_CURRENT = 0.0

#for impedance matching hall voltage
MAX_V_BEFORE_HALL = -2.5
MIN_V_BEFORE_HALL = 2.5   #or y-intercept

MAX_V_AFTER_HALL = 3.3
MIN_V_AFTER_HALL = 0.0

# # default k_b
k_b_1 = 0.08608535912146265
k_b_2 = 0.08533231300575338




def send_function(this_data_1, this_data_2, this_data_3, this_data_4, this_data_5, this_data_6, this_data_7, this_data_8, this_data_9):
    global data_1
    global data_2
    global data_3
    global data_4
    global data_5
    global data_6
    global data_7
    global data_8
    global data_9

    data_1 = this_data_1                 #set and getter
    data_2 = this_data_2
    data_3 = this_data_3
    data_4 = this_data_4
    data_5 = this_data_5
    data_6 = this_data_6
    data_7 = this_data_7
    data_8 = this_data_8
    data_9 = this_data_9

    
    
def send_function_getter():
    return data_1, data_2, data_3, data_4, data_5, data_6, data_7, data_8, data_9


def data_1_getter():


    return int(data_1)

def data_7_getter():
    
    return int(data_7)

def data_hardware_reset_getter():
    global data_9
    return int(data_9)


#setting the flag for Tx event
def send_transmission_event(this_flag_send):
    global flag_send 
    flag_send = this_flag_send
    
    
    
#for use of reusing Tx event from client to the board
def send_transmission_event_getter():
    return flag_send


#setting the flag for Tx event
def running_time_event(this_running_time_flag):
    global running_time_flag
    
    running_time_flag = this_running_time_flag
    
    
#for for use of reusing Rx event from client to the board
def running_time_getter():
    return running_time_flag

#setting the flag for Tx event
def file_name_event(this_file_name_flag):
    global file_name_flag
    
    file_name_flag = this_file_name_flag
    
    
#for use of reusing Rx event from client to the board
def file_name_getter():
    return file_name_flag



def start_flag_rx_event(this_start_flag_rx):
    global start_flag_rx
    
    start_flag_rx = this_start_flag_rx
    
def start_flag_rx_getter():
    return start_flag_rx

def start_flag_send_event(this_start_flag_send):
    global start_flag_send
    
    start_flag_send = this_start_flag_send
    
def start_flag_send_getter():
    return start_flag_send

def stop_button_event(this_stop_button_flag):
    global stop_button_flag
    
    stop_button_flag = this_stop_button_flag
    
def stop_button_getter():
    return stop_button_flag
    
def data_current_start():
    amplitude_1 = data_3
    offset_1 = data_4
    amplitude_2 = data_5
    offset_2 = data_6

    return amplitude_1, offset_1, amplitude_2, offset_2
    

def get_frequency_dac():
    
    global data_2
    
    return data_2           

def get_stop_button_data():
    
    global data_8
    
    return int(data_8)

def combine_bytes_for_buffer(send_1, send_2, send_3, send_4, send_5, send_6, send_7, send_8, send_9):
           
    #ARM Microcontroller is Little Endian, for integer we will be shifting the 
    #bits ourselves but for float, we need to send it little endian preemptively
    
    print("Frequency of DAC", send_2)
    byte_send1 = struct.pack('>I', int(send_1))       
    byte_send2 = struct.pack('<f', float(send_2))             
    byte_send3 = struct.pack('>I', int(send_3))
    byte_send4 = struct.pack('>i', int(send_4))              #offset1
    byte_send5 = struct.pack('>I', int(send_5))
    byte_send6 = struct.pack('>i', int(send_6))              #offset2
    byte_send7 = struct.pack('>I', int(send_7))
    byte_send8 = struct.pack('>I', int(send_8))
    byte_send9 = struct.pack('>I', int(send_9))

    combined_send = b''.join([byte_send1, byte_send2, byte_send3, byte_send4, byte_send5, byte_send6, byte_send7, byte_send8, byte_send9])
    
    return combined_send
   

def change_adc_hall(digital_hall_voltage):

    analogue_hall_voltage = (3.3/RESB_16 * digital_hall_voltage) 
    gradient_analogue  = gradient_calculate(2.5, -2.5, 3.3)
    analogue_hall_voltage_after_impedance_matching = (analogue_hall_voltage*gradient_analogue) + 2.5
     
    return analogue_hall_voltage_after_impedance_matching
    
def change_current_adc(digital_current_values):
    
    analogue_before_adjustment = 3.3/RESB_16*digital_current_values
    gradient_analogue = gradient_calculate(5.0, -5.0 , 3.3)
    
    
    analogue_voltage_after_impedance_matching = analogue_before_adjustment*gradient_analogue + 5.0
    analogue_current_after_impedance_matching = 500/5.0 * analogue_voltage_after_impedance_matching
    return analogue_current_after_impedance_matching

def calibrated_hall_sensors1(hall_voltage, actual_current):
    
    global k_b_1
    calibrated_voltage = hall_voltage - (actual_current*k_b_1)
    return calibrated_voltage

def calibrated_hall_sensors2(hall_voltage, actual_current):
    
    global k_b_2
    calibrated_voltage = hall_voltage - (actual_current*k_b_2)
    return calibrated_voltage


def gradient_calculate(y_intercept, y_axis, x_axis):
     
    gradient =  (y_axis - y_intercept)/x_axis
    
    return float(gradient)

def calculate_running_frequency(input):
    
    C_SR = 37.099
    running_frequency = input/(2*pi*C_SR)
    return running_frequency
        


# def angle_calculate(x1, x2):
#     return numpy.arctan(x1/x2)


# def sin_wave_calibrate(hall_sensor_voltage1, 
#                        hall_sensor_voltage2, current_sensor1, current_sensor2):
    
    
#     get_first_digit =  int(str(current_sensor1[0]))
#     get_second_digit =  int(str(current_sensor2[0]))
    
    
#     return_calibrated_1 = hall_sensor_voltage1 - get_first_digit * sin(2*pi)
#     return_calibrated_1 = hall_sensor_voltage2 - get_second_digit * sin(2*pi)