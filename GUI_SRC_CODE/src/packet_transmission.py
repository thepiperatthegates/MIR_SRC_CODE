#3rd party lazy modules

from math import sin, cos, pi
import struct
import numpy as np



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
CURRENT_COEFF_FIRST_SENSOR_A_VERSION2 = 3.59556
CURRENT_COEFF_FIRST_SENSOR_B_VERSION2 = 0.99575
CURRENT_COEFF_FIRST_SENSOR_C_VERSION2 =  -1.27981e-5
CURRENT_COEFF_FIRST_SENSOR_D_VERSION2 =  -5.72566e-8

#second sensor coefficient version 2
CURRENT_COEFF_SECOND_SENSOR_A_VERSION2 = -6.34285
CURRENT_COEFF_SECOND_SENSOR_B_VERSION2 = 0.99341
CURRENT_COEFF_SECOND_SENSOR_C_VERSION2= 1.18014e-5
CURRENT_COEFF_SECOND_SENSOR_D_VERSION2 = -5.62681e-8


class TxData():

    _data_1 = _data_2 = _data_3 = _data_4 = _data_5 = _data_6 = _data_7 = _data_8 = _data_9 = _data_10 = 0
    
    @property
    def send_data(self):
        return ( self.__class__._data_1, self.__class__._data_2, self.__class__._data_3, self.__class__._data_4, self.__class__._data_5, self.__class__._data_6, 
                self.__class__._data_7, self.__class__._data_8, self.__class__._data_9, self.__class__._data_10)
    
    @send_data.setter
    def send_data(self, values):
        (self.__class__._data_1, self.__class__._data_2, self.__class__._data_3, self.__class__._data_4, 
        self.__class__._data_5, self.__class__._data_6, self.__class__._data_7, self.__class__._data_8, self.__class__._data_9, self.__class__._data_10) = values
        
    @property 
    def data_1(self):
        return self.__class__._data_1 
    
    @data_1.setter
    def data_1(self, val):
        self.__class__._data_1 = val
    
    @property
    def data_2(self):
        return self.__class_._data_2
    
    @data_2.setter
    def data_2(self, val):
        C_SR = 37.099
        running_frequency = float(val)/(2*pi*C_SR)
        self.__class__._data_2 = running_frequency
        
    @property
    def data_2_for_MCU(self):
        return self.__class_._data_2
    
    @data_2_for_MCU.setter
    def data_2_for_MCU(self, val):
        self.__class__._data_2 = val
    
    @property
    def data_current(self):
        return  self.__class__._data_3, self.__class__._data_4, self.__class__._data_5,  self.__class__._data_6
    
    @data_current.setter
    def data_current(self, val):
        self.__class__._data_3, self.__class__._data_4 , self.__class__._data_5 , self.__class__._data_6 = val 
        
    @property 
    def data_4(self):
        return self.__class__._data_4
    
    @data_4.setter
    def data_4(self, val):
        self.__class__._data_4 = val
        
    @property 
    def data_6(self):
        return self.__class__._data_6
    
    @data_6.setter
    def data_6(self, val):
        self.__class__._data_6 = val
        
    @property 
    def data_7(self):
        return self.__class__._data_7
    
    @data_7.setter
    def data_7(self, val):
        self.__class__._data_7 = val
    
    @property 
    def data_8(self):
        return self.__class__._data_8
    
    @data_8.setter
    def data_8(self, val):
        self.__class__._data_8 = val
    
    @property 
    def data_9(self):
        return self.__class__._data_9
    
    @data_9.setter
    def data_9(self, val):
        self.__class__._data_9 = val
    
    @property 
    def data_10(self):
        return self.__class__._data_10
    
    @data_10.setter
    def data_10(self, val):
        self.__class__._data_10 = val
        
    
    def combine_data(self):
        #ARM Microcontroller is Little Endian, for integer we will be shifting the 
        #bits ourselves but for float, we need to send it little endian preemptively
        
        ## TODO: CHANGE BYTE_SEND TO FLOAT 
        print("Frequency of DAC", self.__class__._data_2)
        byte_send1 = struct.pack('<f', float(self.__class__._data_1))       
        byte_send2 = struct.pack('<f', float(self.__class__._data_2))             #running frequency of MCU 
        byte_send3 = struct.pack('<f', float(self.__class__._data_3))              #amplitude1
        byte_send4 = struct.pack('<f', float(self.__class__._data_4))              #offset1
        byte_send5 = struct.pack('<f', float(self.__class__._data_5))              #amplitude2
        byte_send6 = struct.pack('<f', float(self.__class__._data_6))              #offset2
        byte_send7 = struct.pack('>I', int(self.__class__._data_7))                 #if FIR filter is used or not
        byte_send8 = struct.pack('>I', int(self.__class__._data_8))                 #Mode for dc or ac waves (calibration purposes)
        byte_send9 = struct.pack('>I', int(self.__class__._data_9))                 #hardware reset
        byte_send10 = struct.pack('>I', int(self.__class__._data_10))                 #mir mode

        combined_send = b''.join([byte_send1, byte_send2, byte_send3, byte_send4, byte_send5, byte_send6, byte_send7, byte_send8, byte_send9, byte_send10])
        
        return combined_send

    

class TxFlag():
    _flag_tx = False
    
    @property
    def flag_tx(self):
        return self.__class__._flag_tx 
    
    @flag_tx.setter
    def flag_tx(self, val):
        self.__class__._flag_tx = bool(val)

class ProcessUnpackingFlag():
    _flag_process = False
    
    @property
    def flag_process(self):
        return self.__class__._flag_process
    
    @flag_process.setter
    def flag_process(self, val):
        self.__class__._flag_process = bool(val)

class RunningTimeFlag:
    _flag_running_time = False
    
    
    @property
    def flag_running_time(self):
        return self.__class__._flag_running_time 
    
    @flag_running_time.setter
    def flag_running_time(self, value):
        self.__class__._flag_running_time = bool(value)

class StoreArrayGraph():
    _v1_slice = _v2_slice = _i1_slice = _i2_slice = np.array([], dtype=np.uint16)
    
    _angle_permanent_magnet_val = np.array([], dtype=np.float32)
    _angle_magnetic_field_val = np.array([], dtype=np.float32)
    _phase_difference_val = np.array([], dtype=np.float32)
    
    @property
    def v1_slice(self):
        return self.__class__._v1_slice 
    
    @v1_slice .setter
    def v1_slice(self, val):
        self.__class__._v1_slice  = val
    
    @property
    def v2_slice(self):
        return self.__class__._v2_slice
    
    @v2_slice.setter
    def v2_slice(self, val):
        self.__class__._v2_slice = val
        
    @property
    def i1_slice(self):
        return self.__class__._i1_slice
    
    @i1_slice.setter
    def i1_slice(self, val):
        self.__class__._i1_slice = val
        
    @property
    def i2_slice(self):
        return self.__class__._i2_slice
    
    @i2_slice.setter
    def i2_slice(self, val):
        self.__class__._i2_slice = val
        
    @property
    def angle_permanent_magnet_val(self):
        return self.__class__._angle_permanent_magnet_val
    
    @angle_permanent_magnet_val.setter
    def angle_permanent_magnet_val(self, val):
        self.__class__._angle_permanent_magnet_val = val
        
    
    @property
    def angle_magnetic_field_val(self):
        return self.__class__._angle_magnetic_field_val
    
    @angle_magnetic_field_val.setter
    def angle_magnetic_field_val(self, val):
        self.__class__._angle_magnetic_field_val = val
        
        
    @property
    def phase_difference_val(self):
        return self.__class__._phase_difference_val
    
    @phase_difference_val.setter
    def phase_difference_val(self, val):
        self.__class__._phase_difference_val = val
        
    
        

def start_flag_rx_event(this_start_flag_rx):
    global start_flag_rx
    
    start_flag_rx = this_start_flag_rx
    

def stop_button_event(this_stop_button_flag):
    global stop_button_flag
    
    stop_button_flag = this_stop_button_flag
    
def stop_button_getter():
    return stop_button_flag
    


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
        
        current_1 = np.array(current_1, dtype=float)
        current_2 = np.array(current_2, dtype=float)
        hall_1    = np.array(hall_1, dtype=float)
        hall_2    = np.array(hall_2, dtype=float)
        
        offset_1 =float(data_4)
        offset_2 =float(data_6)
        
        #magnetic field angle - magnet angle
        phase_difference = np.arctan2(current_2, current_1) - np.arctan2(hall_2, hall_1)
        
        power_of_2 = np.power((current_1 - offset_1), 2) + np.power((current_2 - offset_2), 2)
        
        magnitude_current = np.sqrt(power_of_2)
        
        total_torque = CALIBRATION_FACTOR * ( DIPOLE_MOMENT
        * COIL_CONSTANT                    # [T/A]
        * magnitude_current / 1000         # mA; -> A, [A]
        * np.sin(phase_difference)   # dimensionless
        )
        
        return total_torque, phase_difference
    
def calculate_radial_frequency(running_frequency):

    return float(2 * np.pi * running_frequency)

class DownsampleEvent():
    
    def __init__(self, parent=None):
        super().__init__()


if __name__ == "__main___":
    calculate_radial_frequency(4)