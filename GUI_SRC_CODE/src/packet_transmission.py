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





#default coefficients

COIL_CONSTANT = 3.097e-3		# in T / A
DIPOLE_MOMENT = 8.594e-3		# in A m^2



#flag for electronics type
ELECTRONICS_FLAG = 0


########################################################### FIRST COIL ###########################################################
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
######################################################################################################################




########################################################### SECOND COIL ###########################################################
# #second sensor coefficient version 2 (<403 mA)
CURRENT_COEFF_FIRST_SENSOR_A_VERSION2 = 1.2231
CURRENT_COEFF_FIRST_SENSOR_B_VERSION2 = 0.97724

#second sensor coefficient version 2 (>403 mA)
CURRENT_COEFF_SECOND_SENSOR_A_VERSION2 = -3.5721
CURRENT_COEFF_SECOND_SENSOR_B_VERSION2  = 0.97735

######################################################################################################################




class TxData():

    _data_1 = _data_2 = _data_3 = _data_4 = _data_5 = _data_6 = _data_7 = _data_8 = _data_9 = _data_10 = _data_offset_creep_1 = _data_offset_creep_2 = 0
    
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
        return self.__class__._data_2
    
    @data_2.setter
    def data_2(self, val):
        C_SR = 37.099
        running_frequency = float(val)/(2*pi*C_SR)
        self.__class__._data_2 = running_frequency
        
    @property
    def data_2_for_MCU(self):
        return self.__class__._data_2
    
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
        
        
    @property 
    def data_offsets_creep(self):
        return self.__class__._data_offset_creep_1, self.__class__._data_offset_creep_2
    

    @data_offsets_creep.setter
    def data_offsets_creep(self, val):
        self.__class__._data_offset_creep_1, self.__class__._data_offset_creep_2 =  val
    
        
    
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

class RemainingTimeForCreepTest():
    _total_time_for_file_save  = 0.0 
    _input_sampling_time = 0.0
    _ime_for_resetting_time = 0.0
    
    
    _start_vector_time = 0.0
    _end_vector_time = 0.0
     
    
    
    @property
    def total_time_for_file_save(self):
        return self.__class__._total_time_for_file_save
    
    @total_time_for_file_save.setter
    def total_time_for_file_save(self, val):
        self.__class__._total_time_for_file_save = val
        
        
    @property
    def input_sampling_time(self):
        return self.__class__._input_sampling_time
    
    @input_sampling_time.setter
    def input_sampling_time(self, val):
        self.__class__._input_sampling_time = val
        
    @property
    def time_for_resetting_time(self):
        self.__class__._time_for_resetting_time = self.__class__._total_time_for_file_save - self.__class__._input_sampling_time
        return self.__class__._time_for_resetting_time
    
    @input_sampling_time.setter
    def time_for_resetting_time(self, val):
        self.__class__._time_for_resetting_time = val
    
    
    @property
    def start_vector_time(self):
        return self.__class__._start_vector_time
    
    @start_vector_time.setter
    def start_vector_time(self, val):
        self.__class__._start_vector_time = val
        
        
    @property
    def end_vector_time(self):
        return self.__class__._end_vector_time
    
    @end_vector_time.setter
    def end_vector_time(self, val):
        self.__class__._end_vector_time = val
        
    

    

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
        
        
class DownSampleSpecificFlag():
    
    _flag_specific_downsample = False
    
    _tot_average = 1
    
    _tot_average_specified = 1
    
    _time_increment = 0.0001
    
    _time_increment_specified = 0.0001
    
    _current_time = 0.0
    
    @property
    def flag_specific_downsample(self):
        return self.__class__._flag_specific_downsample 

    @flag_specific_downsample.setter
    def flag_specific_downsample(self, val):
        self.__class__._flag_specific_downsample = bool(val)
        
        
    @property
    def tot_average(self):
        return self.__class__._tot_average
    
    @tot_average.setter
    def tot_average(self, val):
        self.__class__._tot_average = val 
        
    @property
    def tot_average_specified(self):
        return self.__class__._tot_average_specified
    
    @tot_average_specified.setter
    def tot_average_specified(self, val):
        self.__class__._tot_average_specified = val 
        
    @property
    def time_increment_specified(self):
        return self.__class__._time_increment_specified
    
    @time_increment_specified.setter
    def time_increment_specified(self, val):
        self.__class__._time_increment_specified = val  
        
    @property
    def time_increment(self):
        return self.__class__._time_increment
    
    @time_increment.setter
    def time_increment(self, val):
        self.__class__._time_increment = val
        
    @property
    def current_time(self):
        return self.__class__._current_time
    
    @current_time.setter
    def current_time(self, val):
        self.__class__._current_time = val
        
        

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





class fRCoefficients():

    # Class-level defaults based on the electronics flag
    if ELECTRONICS_FLAG == 1 or ELECTRONICS_FLAG == 0:
        _fr1 = 1.51e-8          # Nm / (rad/s)
        _fr0 = 5.701e-8         # Nm / (rad/s)
        _CALIBRATION_FACTOR = 0.773
    elif ELECTRONICS_FLAG == 2:
        _fr1 = 0.0
        _fr0 = 0.0
        _CALIBRATION_FACTOR = 0.875
    else:
        raise ValueError("Invalid ELECTRONICS_FLAG")

    # --- fr1 ---
    @property
    def fr1(self):
        return self.__class__._fr1

    @fr1.setter
    def fr1(self, value: float):
        self.__class__._fr1 = float(value)

    # --- fr0 ---
    @property
    def fr0(self):
        return self.__class__._fr0

    @fr0.setter
    def fr0(self, value: float):
        self.__class__._fr0 = float(value)

    # --- calibration factor ---
    @property
    def CALIBRATION_FACTOR(self):
        return self.__class__._CALIBRATION_FACTOR

    @CALIBRATION_FACTOR.setter
    def CALIBRATION_FACTOR(self, value: float):
        self.__class__._CALIBRATION_FACTOR= float(value)



class kbCoefficient():

    #default calibration factor k_b 
    
    if ELECTRONICS_FLAG == 1 or ELECTRONICS_FLAG == 0:
        _k_b_1 = 0.083597946
        _k_b_2 = 0.084535931
    elif ELECTRONICS_FLAG == 2:
        _k_b_1 = 0.16423985101661842
        _k_b_2 = 0.1666737395533449
    
    
    @property
    def k_b_1(self):
        return self.__class__._k_b_1
    
    @k_b_1.setter
    def k_b_1(self, val):
        self.__class__._k_b_1 = val 
        
        
    @property
    def k_b_2(self):
        return self.__class__._k_b_2
    
    @k_b_2.setter
    def k_b_2(self, val):
        self.__class__._k_b_2 = val 


class VoltageNormaliseCoefficient():

    _amplitude_voltage_1 = 0.0
    _zero_offset_voltage_1 = 0.0

    _amplitude_voltage_2 = 0.0
    _zero_offset_voltage_2 = 0.0

    @property
    def amplitude_voltage_1(self):
        return self.__class__._amplitude_voltage_1

    @amplitude_voltage_1.setter
    def amplitude_voltage_1(self, val):
        self.__class__._amplitude_voltage_1 = val

    @property
    def zero_offset_voltage_1(self):
        return self.__class__._zero_offset_voltage_1

    @zero_offset_voltage_1.setter
    def zero_offset_voltage_1(self, val):
        self.__class__._zero_offset_voltage_1 = val

    @property
    def amplitude_voltage_2(self):
        return self.__class__._amplitude_voltage_2

    @amplitude_voltage_2.setter
    def amplitude_voltage_2(self, val):
        self.__class__._amplitude_voltage_2 = val

    @property
    def zero_offset_voltage_2(self):
        return self.__class__._zero_offset_voltage_2

    @zero_offset_voltage_2.setter
    def zero_offset_voltage_2(self, val):
        self.__class__._zero_offset_voltage_2 = val




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
    return gradient

def calibrated_hall_sensors1(k_b_1, hall_voltage, actual_current):
    
    calibrated_voltage = hall_voltage - (actual_current*k_b_1)
    return calibrated_voltage

def calibrated_hall_sensors2(k_b_2, hall_voltage, actual_current):
    
    
    calibrated_voltage = hall_voltage - (actual_current*k_b_2)
    return calibrated_voltage



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
        output = CURRENT_COEFF_FIRST_SENSOR_A_VERSION2 + CURRENT_COEFF_FIRST_SENSOR_B_VERSION2 * input
        
    return output   


def calibration_input_coil_2(input):
    global ELECTRONICS_FLAG
    
    
    if ELECTRONICS_FLAG == 1:
        output = CURRENT_COEFF_SECOND_SENSOR_A_VERSION1 + CURRENT_COEFF_SECOND_SENSOR_B_VERSION1 * input + CURRENT_COEFF_SECOND_SENSOR_C_VERSION1 * pow(input, 2) + CURRENT_COEFF_SECOND_SENSOR_D_VERSION1 * pow(input, 3)
    elif ELECTRONICS_FLAG == 2:
        output = CURRENT_COEFF_SECOND_SENSOR_A_VERSION2 + CURRENT_COEFF_SECOND_SENSOR_B_VERSION2 * input 
            
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
        
        global DIPOLE_MOMENT, COIL_CONSTANT
        
        current_1 = np.array(current_1, dtype=float)
        current_2 = np.array(current_2, dtype=float)
        hall_1    = np.array(hall_1, dtype=float)
        hall_2    = np.array(hall_2, dtype=float)
        
        offset_1 =  float(data_4)
        offset_2 =  float(data_6)
        
        #magnetic field angle - magnet angle
        phase_difference = np.arctan2(current_2, current_1) - np.arctan2(hall_2, hall_1)
        
        power_of_2 = np.power((current_1 - offset_1), 2) + np.power((current_2 - offset_2), 2)
        
        magnitude_current = np.sqrt(power_of_2)
        
        worker_cal = fRCoefficients()
        
        total_torque = worker_cal.CALIBRATION_FACTOR * ( DIPOLE_MOMENT
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