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
data_10 = 0


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

#default calibration factor k_b
k_b_1 = 0.083597946
k_b_2 = 0.084535931

#default friction coefficient 
CALIBRATION_FACTOR = 0.773              # torque calibration no units (K)
fr1 = 1.51e-8 # in Nm / (rad /s)
fr0 = 5.701e-8 # in Nm / (rad /s)

#default coefficients

COIL_CONSTANT = 3.097e-3		# in T / A
DIPOLE_MOMENT = 8.594e-3		# in A m^2

#offset from main.py
offset_1 = 0.0
offset_2 = 0.0




#flag for electronics type
ELECTRONICS_FLAG = 0


#first sensor coefficient version 1
CURRENT_COEFF_FIRST_SENSOR_A_VERSION1 = -4.61934
CURRENT_COEFF_FIRST_SENSOR_B_VERSION1 = 0.99182
CURRENT_COEFF_FIRST_SENSOR_C_VERSION1 =  -4.24388e-6
CURRENT_COEFF_FIRST_SENSOR_D_VERSION1 =  -5.40711e-8

#second sensor coefficient version 1
CURRENT_COEFF_SECOND_SENSOR_A_VERSION1 = 3.86547
CURRENT_COEFF_SECOND_SENSOR_B_VERSION1 = 0.98655
CURRENT_COEFF_SECOND_SENSOR_C_VERSION1= 3.02401e-6
CURRENT_COEFF_SECOND_SENSOR_D_VERSION1 = -3.98871e-8

#first sensor coefficient version 2
CURRENT_COEFF_FIRST_SENSOR_A_VERSION2 = 0
CURRENT_COEFF_FIRST_SENSOR_B_VERSION2 = 0
CURRENT_COEFF_FIRST_SENSOR_C_VERSION2 =  0
CURRENT_COEFF_FIRST_SENSOR_D_VERSION2 =  0

#second sensor coefficient version 2
CURRENT_COEFF_SECOND_SENSOR_A_VERSION2 = 0
CURRENT_COEFF_SECOND_SENSOR_B_VERSION2 = 0
CURRENT_COEFF_SECOND_SENSOR_C_VERSION2= 0
CURRENT_COEFF_SECOND_SENSOR_D_VERSION2 = 0


def send_function(this_data_1, this_data_2, this_data_3, this_data_4, this_data_5, this_data_6, this_data_7, this_data_8, this_data_9, this_data_10):
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

    data_1 = this_data_1                 #set and getter
    data_2 = this_data_2
    data_3 = this_data_3
    data_4 = this_data_4
    data_5 = this_data_5
    data_6 = this_data_6
    data_7 = this_data_7
    data_8 = this_data_8
    data_9 = this_data_9
    data_10 = this_data_10

    
    
def send_function_getter():
    return data_1, data_2, data_3, data_4, data_5, data_6, data_7, data_8, data_9, data_10


def data_1_getter():


    return float(data_1)

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

def get_mir_mode():
    global data_10 
    
    return int(data_10)

def get_offsets():
    global data_4, data_6 
    
    print(data_4)
    return float(data_4), float(data_6)
def combine_bytes_for_buffer(send_1, send_2, send_3, send_4, send_5, send_6, send_7, send_8, send_9, send_10):
           
    #ARM Microcontroller is Little Endian, for integer we will be shifting the 
    #bits ourselves but for float, we need to send it little endian preemptively
    
    
    ## TODO: CHANGE BYTE_SEND TO FLOAT 
    print("Frequency of DAC", send_2)
    byte_send1 = struct.pack('<f', float(send_1))       
    byte_send2 = struct.pack('<f', float(send_2))             #running frequency of MCU 
    byte_send3 = struct.pack('<f', float(send_3))              #amplitude1
    byte_send4 = struct.pack('<f', float(send_4))              #offset1
    byte_send5 = struct.pack('<f', float(send_5))              #amplitude2
    byte_send6 = struct.pack('<f', float(send_6))              #offset2
    byte_send7 = struct.pack('>I', int(send_7))                 
    byte_send8 = struct.pack('>I', int(send_8))                 #Mode for dc or ac waves (calibration purposes)
    byte_send9 = struct.pack('>I', int(send_9))                 #hardware reset
    byte_send10 = struct.pack('>I', int(send_10))                 #mir mode

    combined_send = b''.join([byte_send1, byte_send2, byte_send3, byte_send4, byte_send5, byte_send6, byte_send7, byte_send8, byte_send9, byte_send10])
    
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


def gradient_calculate(y_intercept, y_axis, x_axis):
     
    gradient =  (y_axis - y_intercept)/x_axis
    return float(gradient)

def calibrated_hall_sensors1(hall_voltage, actual_current):
    
    global k_b_1
    calibrated_voltage = hall_voltage - (actual_current*k_b_1)
    return calibrated_voltage

def calibrated_hall_sensors2(hall_voltage, actual_current):
    
    global k_b_2
    calibrated_voltage = hall_voltage - (actual_current*k_b_2)
    return calibrated_voltage

def get_fr_coefficient():
    
    global fr0, fr1
    
    return fr0, fr1


def calculate_running_frequency(input):
    
    C_SR = 37.099
    running_frequency = float(input)/(2*pi*C_SR)
    return float(running_frequency)


def set_electronics_flag(flag):
    global ELECTRONICS_FLAG
    
    ELECTRONICS_FLAG = flag 
    
    
def get_electronics_flag():
    
    global ELECTRONICS_FLAG
    
    return ELECTRONICS_FLAG

def calibration_input_coil_1(input):
    
    global ELECTRONICS_FLAG
    
    if ELECTRONICS_FLAG == 1:
        output = CURRENT_COEFF_FIRST_SENSOR_A_VERSION1 + CURRENT_COEFF_FIRST_SENSOR_B_VERSION1 * input + CURRENT_COEFF_FIRST_SENSOR_C_VERSION1 * pow(input, 2) +  CURRENT_COEFF_FIRST_SENSOR_D_VERSION1 * pow(input, 3)
    elif ELECTRONICS_FLAG == 2:
        output = CURRENT_COEFF_FIRST_SENSOR_A_VERSION2 + CURRENT_COEFF_FIRST_SENSOR_B_VERSION2 * input + CURRENT_COEFF_FIRST_SENSOR_C_VERSION2 * pow(input, 2) +  CURRENT_COEFF_FIRST_SENSOR_D_VERSION2 * pow(input, 3)
    return output   


def calibration_input_coil_2(input):
    global ELECTRONICS_FLAG
    
    
    if ELECTRONICS_FLAG == 1:
        output = CURRENT_COEFF_SECOND_SENSOR_A_VERSION1 + CURRENT_COEFF_SECOND_SENSOR_B_VERSION1 * input + CURRENT_COEFF_SECOND_SENSOR_C_VERSION1 * pow(input, 2) + CURRENT_COEFF_SECOND_SENSOR_D_VERSION1 * pow(input, 3)
    elif ELECTRONICS_FLAG == 2:
        output = CURRENT_COEFF_SECOND_SENSOR_A_VERSION2 + CURRENT_COEFF_SECOND_SENSOR_B_VERSION2 * input + CURRENT_COEFF_SECOND_SENSOR_C_VERSION2 * pow(input, 2) +  CURRENT_COEFF_SECOND_SENSOR_D_VERSION2 * pow(input, 3)
    return output   


# def angle_calculate(x1, x2):
#     return numpy.arctan(x1/x2)


# def sin_wave_calibrate(hall_sensor_voltage1, 
#                        hall_sensor_voltage2, current_sensor1, current_sensor2):
    
    
#     get_first_digit =  int(str(current_sensor1[0]))
#     get_second_digit =  int(str(current_sensor2[0]))
    
    
#     return_calibrated_1 = hall_sensor_voltage1 - get_first_digit * sin(2*pi)
#     return_calibrated_1 = hall_sensor_voltage2 - get_second_digit * sin(2*pi)


                
def calculate_torque_fR( current_1, current_2, hall_1, hall_2, data_4, data_6):
        """
        Calculate the torque based on Hall sensor and current measurements.

        This method computes the applied torque using the phase difference between
        the magnetic field (from the current coils) and the magnet (from
        the Hall sensors). It applies calibration factors, coil constants,
        and the dipole moment to convert the measured currents to torque.

        :param current_1: First component of the current measurement.
        :type current_1: float or np.ndarray
        :param current_2:ray(hall_2, dtype=float)
        
        print("current_2:", current_2)
        global data_4,  Second component of the current measurement.
        :type current_2: float or np.ndarray
        :param hall_1: First Hall sensor reading.
        :type hall_1: float or np.ndarray
        :param hall_2: Second Hall sensor reading.
        :type hall_2: float or np.ndarray

        :return: Calculated torque.
        :rtype: float or np.ndarray
        """
        
        global CALIBRATION_FACTOR, DIPOLE_MOMENT, COIL_CONSTANT
        
        current_1 = numpy.array(current_1, dtype=float)
        current_2 = numpy.array(current_2, dtype=float)
        hall_1    = numpy.array(hall_1, dtype=float)
        hall_2    = numpy.array(hall_2, dtype=float)
        
        offset_1 =float(data_4)
        offset_2 =float(data_6)
        
        #magnetic field angle - magnet angle
        phase_difference = numpy.arctan2(current_2, current_1) - numpy.arctan2(hall_2, hall_1)
        
        power_of_2 = numpy.power((current_1 - offset_1), 2) + numpy.power((current_2 - offset_2), 2)
        
        magnitude_current = numpy.sqrt(power_of_2)
        
        total_torque = CALIBRATION_FACTOR * ( DIPOLE_MOMENT
        * COIL_CONSTANT                    # [T/A]
        * magnitude_current / 1000         # mA; -> A, [A]
        * numpy.sin(phase_difference)   # dimensionless
        )
        
        return total_torque, phase_difference
    
def calculate_radial_frequency(running_frequency):

    return float(2 * numpy.pi * running_frequency)




if __name__ == "__main___":
    send_transmission_event(1)