#3rd party lazy modules

from math import  pi
import os
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

MIN_V_AFTER_HALL = 0.0

#default coefficients

COIL_CONSTANT = 3.097e-3		# in T / A
DIPOLE_MOMENT = 8.594e-3		# in A m^2



#flag for electronics type
ELECTRONICS_FLAG = 0


class TxData():

    _data_1 = _data_2 = _data_3 = _data_4 = _data_5 = _data_6 = _data_7 = _data_8 = _data_9 = _data_10 = _data_11 = _data_offset_creep_1 = _data_offset_creep_2 = 0
    _time_acquisition = 0
    
    FRAME_HEADER_1     =  0xCA
    FRAME_HEADER_INPUT = 0xCB
    FRAME_HEADER_COMP  = 0xCD
        
    @property
    def send_data(self):
        return ( self.__class__._data_1, self.__class__._data_2, self.__class__._data_3, self.__class__._data_4, self.__class__._data_5, self.__class__._data_6, 
                self.__class__._data_7, self.__class__._data_8, self.__class__._data_9, self.__class__._data_10)
    
    @send_data.setter
    def send_data(self, values):
        (self.__class__._data_1, self.__class__._data_2, self.__class__._data_3, self.__class__._data_4, 
        self.__class__._data_5, self.__class__._data_6, self.__class__._data_7, self.__class__._data_8, self.__class__._data_9, self.__class__._data_10) = values
        
    @property
    def time_acquisition(self):
        return self.__class__._time_acquisition
    
    @time_acquisition.setter
    def time_acquisition(self, val):
        self.__class__._time_acquisition = float(val)
        
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
        # ---------------------- we will be taking the desired shear rate here directly ----------------------
        self.__class__._data_2 = val
        
    @property
    def data_2_CSR(self):
        return self.__class__._data_2
    
    @data_2_CSR.setter
    def data_2_CSR(self, val):
        C_SR = 37.099
        running_frequency = float(val)/(2*pi*C_SR)
        # ---------------------- we will be taking the desired shear rate here directly ----------------------
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
    def data_3(self):
        return self.__class__._data_3
    
    @data_3.setter
    def data_3(self, val):
        self.__class__._data_3 = val
        
        
    @property 
    def data_4(self):
        return self.__class__._data_4
    
    @data_4.setter
    def data_4(self, val):
        self.__class__._data_4 = val
        
    @property 
    def data_5(self):
        return self.__class__._data_5
    
    @data_5.setter
    def data_5(self, val):
        self.__class__._data_5 = val
        
        
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
    def data_11(self):
        return self.__class__._data_11 
    
    @data_11.setter 
    def data_11(self, val):
        self.__class__._data_11 = val
        
        
    @property 
    def data_offsets(self):
        return self.__class__._data_4, self.__class__._data_6
    
    def combine_input_data(self):
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
        byte_send11 = struct.pack('<f', float(self.__class__._data_11))             # shear stress/torque reference

        payload = b''.join([byte_send1, byte_send2, byte_send3, 
                                byte_send4, byte_send5, byte_send6, byte_send7, byte_send8, byte_send9, byte_send10, byte_send11])
        
        # sending data with input frameheader
        return bytes([self.__class__.FRAME_HEADER_1, self.__class__.FRAME_HEADER_INPUT]) + payload
    
    def combine_additional_data(self, data1, data2, data3, data4, data5):

        #note: <I is used here even if data is uint16_t because of deserializing algorithm on MCU.
        byte_1 = struct.pack('>I', int(data1))
        byte_2 = struct.pack('>I', int(data2))
        byte_3 = struct.pack('>I', int(data3))
        byte_4 = struct.pack('>I', int(data4))
        byte_5 = struct.pack('>I', int(data5))
        
        payload = b''.join([byte_1, byte_2, byte_3, byte_4, byte_5])
        
        return bytes([self.__class__.FRAME_HEADER_1, self.__class__.FRAME_HEADER_COMP]) + payload
    
class TxFlag():
    _flag_tx = False
    _flag_init_tx = False
    
    @property
    def flag_tx(self):
        return self.__class__._flag_tx 
    
    @flag_tx.setter
    def flag_tx(self, val):
        self.__class__._flag_tx = bool(val)
        
    @property
    def flag_init_tx(self):
        return self.__class__._flag_init_tx 
    
    @flag_init_tx.setter
    def flag_init_tx(self, val):
        self.__class__._flag_init_tx = bool(val)

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
    _flag_norm_save = False
    
    
    @property
    def flag_running_time(self):
        return self.__class__._flag_running_time 
    
    @flag_running_time.setter
    def flag_running_time(self, value):
        self.__class__._flag_running_time = bool(value)
        
    @property
    def flag_norm_save(self):
        return self.__class__._flag_norm_save
    
    @flag_norm_save.setter
    def flag_norm_save(self, value):
        self.__class__._flag_norm_save = bool(value)
        
        
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
    _actual_torque_val =  np.array([], dtype=np.float32)
    
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
        
    @property
    def actual_torque_val(self):
        return self.__class__._actual_torque_val
    
    @actual_torque_val.setter
    def actual_torque_val(self, val):
        self.__class__._actual_torque_val = val


class fRCoefficients:
    _initialized = False

    # class-level shared variables (defaults)
    _fr1 = None
    _fr0 = None
    _CALIBRATION_FACTOR = None

    @classmethod
    def _initialize(cls):
        """Run once, based on ELECTRONICS_FLAG."""
        global ELECTRONICS_FLAG
        if cls._initialized:
            return

        if ELECTRONICS_FLAG in (0, 1):
            cls._fr1 = 1.51e-8
            cls._fr0 = -8.577e-08
            cls._CALIBRATION_FACTOR = 0.773
        elif ELECTRONICS_FLAG == 2:
            cls._fr1 = 2.613e-08
            cls._fr0 = 1.186e-07
            cls._CALIBRATION_FACTOR = 0.875
        else:
            raise ValueError(f"Invalid ELECTRONICS_FLAG = {ELECTRONICS_FLAG}")

        cls._initialized = True
        
    
    # ----- Reload ---------
    @classmethod
    def reload(cls):
        cls._initialized = False
        cls._initialize()
        
        
    # ---------- Properties ----------
    @property
    def fr1(self):
        type(self)._initialize()
        return type(self)._fr1

    @fr1.setter
    def fr1(self, value):
        type(self)._initialize()
        type(self)._fr1 = float(value)

    @property
    def fr0(self):
        type(self)._initialize()
        return type(self)._fr0

    @fr0.setter
    def fr0(self, value):
        type(self)._initialize()
        type(self)._fr0 = float(value)

    @property
    def CALIBRATION_FACTOR(self):
        type(self)._initialize()
        return type(self)._CALIBRATION_FACTOR

    @CALIBRATION_FACTOR.setter
    def CALIBRATION_FACTOR(self, value):
        type(self)._initialize()
        type(self)._CALIBRATION_FACTOR = float(value)


class kbCoefficient:
    _initialized = False

    _k_b_1 = 0.0
    _k_b_2 = 0.0

    @classmethod
    def _initialize(cls):
        """Run once based on the global ELECTRONICS_FLAG."""
        global ELECTRONICS_FLAG
        
        if cls._initialized:
            return
        
        project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        
        filepath = os.path.join(project_root, "files", "k_b_coefficient.csv")
        
        user_input = ELECTRONICS_FLAG
        
        #Load CSV data along with headers
        data = np.genfromtxt(filepath, delimiter=";", names=True)
        
        #Boolean comparison in the first column
        index = data["ELECTRONICS_FLAG"] == user_input 
        if not np.any(index):
            raise ValueError(f"No matching ELECTRONICS_FLAG = {user_input} in CSV")
        
        row = data[index][0]
        
        #get the data
        cls._k_b_1 = float(row["k_b_1"])
        cls._k_b_2 = float(row["k_b_2"])

        cls._initialized = True
        
        
    # ---- reload ----
    @classmethod
    def reload(cls):
        cls._initialized = False
        cls._initialize()

    # ---- k_b_1 ----
    @property
    def k_b_1(self):
        type(self)._initialize()
        return type(self)._k_b_1

    @k_b_1.setter
    def k_b_1(self, val):
        type(self)._initialize()
        type(self)._k_b_1 = float(val)

    # ---- k_b_2 ----
    @property
    def k_b_2(self):
        type(self)._initialize()
        return type(self)._k_b_2

    @k_b_2.setter
    def k_b_2(self, val):
        type(self)._initialize()
        type(self)._k_b_2 = float(val)
        
        
        
class VoltageNormaliseCoefficient:
    _initialized = False
    
    _min_hall_1 = 0.0
    _max_hall_1 = 0.0
    _min_hall_2= 0.0
    _max_hall_2 = 0.0
    
    @classmethod 
    def _initialize(cls):
        
        if cls._initialized:
            return

        project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        filepath = os.path.join(project_root, "files", "normalise_voltage_constant.csv")

        # If file exists → load values (legacy code)
        if os.path.exists(filepath):
            data = np.genfromtxt(filepath, delimiter=';', names=True)

            cls._min_hall_1 = float(data["min_hall_1_v"])
            # print("amp_voltage_1", cls._min_hall_1)
            cls._max_hall_1 = float(data["max_hall_1_v"])
            # print("amp_voltage_zero_offset_1_v", cls._max_hall_1 )
            cls._min_hall_2= float(data["min_hall_2_v"])
            # print("amp_voltage_2", cls._amp_voltage_2)
            cls._max_hall_2 = float(data["max_hall_2_v"])
            # print("amp_voltage_zero_offset_2_v", cls._max_hall_2 )
            
        
        
        print("From files amp 1: ", cls._min_hall_1)
        print("From _max_hall_1: ", cls._max_hall_1)
        cls._initialized = True
        
        
    # ---- reload ----
    @classmethod
    def reload(cls):
        cls._initialized = False
        cls._initialize()
        
    # ---- voltage 1 ----
        
    @property
    def min_hall_1(self):
        type(self)._initialize()    
        return type(self)._min_hall_1
    
    @min_hall_1.setter
    def min_hall_1(self, val):
        type(self)._initialize()
        type(self)._min_hall_1= float(val)
        
    
    @property
    def max_hall_1(self):
        type(self)._initialize()    
        return type(self)._max_hall_1
    
    @max_hall_1.setter
    def max_hall_1(self, val):
        type(self)._initialize()
        type(self)._max_hall_1 = float(val)
        
    # ---- voltage 2----
        
    @property
    def min_hall_2(self):
        type(self)._initialize()    
        return type(self)._min_hall_2
    
    @min_hall_2.setter
    def min_hall_2(self, val):
        type(self)._initialize()
        type(self)._min_hall_2= float(val)
        
    
    @property
    def max_hall_2(self):
        type(self)._initialize()    
        return type(self)._max_hall_2
    
    @max_hall_2.setter
    def max_hall_2(self, val):
        type(self)._initialize()
        type(self)._max_hall_2 = float(val)
    



def stop_button_event(this_stop_button_flag):
    global stop_button_flag
    
    stop_button_flag = this_stop_button_flag
    
def stop_button_getter():
    return stop_button_flag



_HALL_SCALE  = -5.0    / RESB_16   # [V / LSB]
_HALL_OFFSET =  2.5                # [V]

_CURRENT_SCALE  = -1000.0 / RESB_16  # [mA / LSB]
_CURRENT_OFFSET =  500.0             # [mA]


def change_adc_hall(digital_hall_voltage):
    """Accepts scalar or numpy array. Returns Hall voltage [V]."""
    return np.multiply(_HALL_SCALE, digital_hall_voltage) + _HALL_OFFSET

def change_current_adc(digital_current_values):
    """Accepts scalar or numpy array. Returns coil current [mA]."""
    return np.multiply(_CURRENT_SCALE, digital_current_values) + _CURRENT_OFFSET


def calibrated_hall_sensors1(k_b_1, norm_voltage, actual_current, min_hall, max_hall):
    
    adc_raw = (norm_voltage + 1.0) * (max_hall - min_hall ) / 2.0 + min_hall
    hall_voltage = change_adc_hall(adc_raw)
    #---- Apply kb calibration ----
    hall_voltage = hall_voltage - (actual_current * k_b_1)
    return hall_voltage

def calibrated_hall_sensors2(k_b_2, norm_voltage, actual_current, min_hall, max_hall):
    
    adc_raw = (norm_voltage + 1.0) * (max_hall - min_hall ) / 2.0 + min_hall
    hall_voltage = change_adc_hall(adc_raw)
    #---- Apply kb calibration ----
    hall_voltage = hall_voltage - (actual_current * k_b_2)
    return hall_voltage


def set_electronics_flag(flag):
    global ELECTRONICS_FLAG
    
    ELECTRONICS_FLAG = flag 
    
    
def get_electronics_flag():
    
    global ELECTRONICS_FLAG
    
    return ELECTRONICS_FLAG

# ---------- FIRST ELECTRONIC ----------
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
#----------------------------------------

# ---------- SECOND ELECTRONIC ----------
# #second sensor coefficient version 2 (<403 mA)
CURRENT_COEFF_FIRST_SENSOR_A_VERSION2 = 1.2231
CURRENT_COEFF_FIRST_SENSOR_B_VERSION2 = 0.97724

#second sensor coefficient version 2 (>403 mA)
CURRENT_COEFF_SECOND_SENSOR_A_VERSION2 = -3.5721
CURRENT_COEFF_SECOND_SENSOR_B_VERSION2  = 0.97735
#----------------------------------------



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


def set_popout_text(arg,calculate_final_fR = 0.0, k_b_1 = 0.0, k_b_2 = 0.0):
    
    text = None
    
    if arg == 1:
        text = "Successful"
    elif arg == 2:
        text = f"k b 1 = {k_b_1}\n"f"k_b_2 = {k_b_2}"
    elif arg == 3:
        text = f"Friction coefficient measurement is starting innit"
    elif arg == 4:
        text = f"f_R: {np.poly1d(calculate_final_fR)}"
    elif arg == 5:
        text = "Invalid sample frequency! Please do not use any value starting with 3, 7 or 9!"
    elif arg == 6:
        text = "Normalise parameters completed!"
        
    return text

class DownsampleEvent():
    def __init__(self, parent=None):
        super().__init__()


if __name__ == "__main___":
    calculate_radial_frequency(4)