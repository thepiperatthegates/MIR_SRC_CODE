#---------------------- functions for socket backend ----------------------

from . import device_state

import numpy as np
import os
import sys
import threading
import time
import multiprocessing
import serial
import struct

import queue

tx_queue = queue.Queue()
#--------------------------- queue init ---------------------------
q_to_process = multiprocessing.Queue()
q_to_graph = multiprocessing.Queue()
q_to_csv = multiprocessing.Queue()

q_to_norm = multiprocessing.Queue()
#-------------------------------------------------------------------

#---------------------- setter for port_name ----------------------
port_name = None 

current_time = None
status_connection = None


count_time = 0
flag_for_process = None
flag_for_downsampling = None
flag_test = None

p1 = None

worker_data_flag = device_state.RunningTimeFlag()
restart_event = multiprocessing.Event()

# -------------- Data transmission CONSTANTS -------------------------

# --- Data Type Sizes (in Byte) ---
UINT8_SIZE   = 1
UINT16_SIZE  = 2
FLOAT32_SIZE = 4

# --- Frame Configuration ---
# Frame structure: [Header (2 bytes)] + [Payload]
DMA_BUFFER_SIZE      = 2
HEADER_SIZE          = 2 * UINT8_SIZE
PAYLOAD_DATA_SIZE    = (2 * UINT16_SIZE) + (4 * FLOAT32_SIZE)
BYTES_PER_SAMPLE     = HEADER_SIZE + PAYLOAD_DATA_SIZE
TOTAL_ONE_CYCLE_BYTES    = BYTES_PER_SAMPLE * DMA_BUFFER_SIZE


# --- Timing & Frequency ---
# Calculations for 10kHz base frequency (0.0001s)
SAMPLE_PERIOD        = 0.0001 * DMA_BUFFER_SIZE
TOT_COUNT_ACCUMULATE_RECV_IN_1_SEC   = int(0.1 / SAMPLE_PERIOD)

# ---------------------- for total count receiving from socket (depends if we want 0.5s, 1s or 2s) ----------------------
TOT_COUNT_ACCUMULATE_RECV_IN_1_SEC_FRONTEND =   int(0.1 / SAMPLE_PERIOD)

# ---------------------- DATA COLUMNS FROM SENSORS ---------------------
num_columns = 6



def init_queues():
    global q_to_process, q_to_graph, q_to_csv
    q_to_process = multiprocessing.Queue()
    q_to_graph = multiprocessing.Queue()
    q_to_csv = multiprocessing.Queue()
    print("Multiprocessing queues initialized.")


def port_name_setter(this_port_name):
    global port_name 
    
    port_name = this_port_name

#---------------------- start socket connection for USB ----------------------
def socket_start_connect():
    global port_name

    port_num = None 
    baud_rate = 128000
    
    # Detect platform and set port
    if sys.platform == 'darwin':        # macOS
        port_num = '/dev/tty.usbmodem3776345D32331'
    elif sys.platform == 'win32':       # Windows
        port_num = port_name  # must be defined elsewhere
    elif sys.platform == 'linux':       # Linux
        for options in ['/dev/ttyACM1', '/dev/ttyACM2', '/dev/ttyACM3']:
            if os.path.exists(options):
                port_num = options
                break
    try:
        ser = serial.Serial(port=port_num, baudrate=baud_rate,timeout=None)
        print("Connecting to the board")
        print("Successful connection")
        
        status_connection = True
    except Exception as e:
        print("Cannot connect with USB serial port!:", e)
        print(port_name)
        socket_start_connect()  #RECURSIVE TO TRY AGAIN
        status_connection = False
        
    return ser

# ---------------------- start creating two separate thread ----------------------
def thread_start():
    """
    Start the main communication threads for serial data transmission.

    This function establishes a socket/serial connection and manages two
    separate threads for monitoring and sending data:

    - `recv_thread`: Runs in a background thread to continuously monitor 
      incoming data from the serial port.
    - `send_thread`: Spawned whenever the `start_flag_send` is set. It handles
      sending packets over the established connection.

    Workflow:
        1. Connect to the serial/socket using `socket_start_connect()`.
        2. Start a receiving thread (`recv_thread`) to listen for incoming data.
        3. In a continuous loop:
            - Check if `start_flag_send` is set.
            - If set, reset it and start a sending thread (`send_thread`).
            - Sleep briefly to prevent busy-waiting.
    
    Notes:
        - The receiving thread is persistent and always active.
        - The sending thread is created on demand when the send flag is raised.
        - A small delay (`time.sleep(0.001)`) is used to reduce CPU usage.
    """
    ser1 = socket_start_connect()
    worker_kb_property = device_state.kbCoefficient()
    worker_specific_downsampling = device_state.DownSampleSpecificFlag()
    worker_normalisation  = device_state.VoltageNormaliseCoefficient()
    
    #Event for run time receiving data from Serial Porte
    thread_recv = threading.Thread(target=recv_thread, args=(ser1,worker_kb_property, worker_specific_downsampling))
    thread_recv.start()

    while True:
        tx_type = tx_queue.get()
        if tx_type == "input":
            thread_send = threading.Thread(target=send_thread, daemon=False, args=(ser1, "input"))
            thread_send.start()
        elif tx_type == "norm":
            thread_send = threading.Thread(target=send_thread, daemon=False, args=(ser1, "norm", worker_normalisation.min_hall_1, worker_normalisation.max_hall_1, 
                                        worker_normalisation.min_hall_2, worker_normalisation.max_hall_2))
            thread_send.start()

def recv_thread(ser1, worker_kb_property, worker_specific_downsampling):
    
    #----- pasing worker objects ------------
    worker_process_flag = device_state.ProcessUnpackingFlag()
    worker_normalise_properties = device_state.VoltageNormaliseCoefficient()

    restart_event = multiprocessing.Event()

    count = 0
    while True:
        try:
            received_data = b''

            try:
                while count < TOT_COUNT_ACCUMULATE_RECV_IN_1_SEC:
                    count += 1
                    chunk = b''
                    while len(chunk) < TOTAL_ONE_CYCLE_BYTES:
                        chunk += ser1.read(TOTAL_ONE_CYCLE_BYTES - len(chunk))
                    received_data += chunk

                count = 0

                # -----  Start live graph process for the first time --------
                if not worker_process_flag.flag_process and received_data:
                    p1 = multiprocessing.Process(target=start_process_live_graph,
                                                  args=(q_to_process, q_to_graph, q_to_csv, q_to_norm, restart_event))
                    p1.start()
                    worker_process_flag.flag_process = True

                q_to_process.put(received_data)

                # ---- Non-blocking get with timeout to avoid freeze ----
                sensor_data_recv = None
                try:
                    # Non blocking queue get (since it might takes a while for firmware to react)
                    sensor_data_recv = q_to_csv.get(timeout=1.0)  
                except Exception:
                    pass  # timeout hit, restart the process

                # ------ Restart process if norm triggered exit --------- 
                if not p1.is_alive() and worker_process_flag.flag_process:
                    print("[RESTART] Restarting start_process_live_graph...")
                    restart_event.clear()
                    #-----  Drain all queues before restarting so no data conflict -----
                    drain_queue(q_to_process)
                    drain_queue(q_to_graph)
                    drain_queue(q_to_csv)
                    drain_queue(q_to_norm)
                    
                    p1 = multiprocessing.Process(target=start_process_live_graph,
                                                  args=(q_to_process, q_to_graph, q_to_csv, q_to_norm, restart_event))
                    p1.start()

                
                # ---- Saving data function -------
                if worker_data_flag.flag_running_time:
                    if sensor_data_recv:
                        save_sensors_data_to_csv(sensor_data_recv, worker_kb_property, worker_specific_downsampling,
                                                  worker_normalise_properties, num_columns=num_columns)
                        sensor_data_recv = None

                if worker_data_flag.flag_norm_save:
                    norm_values_recv = q_to_norm.get()
                    print("Here?")
                    if norm_values_recv:
                        save_norm_data_to_csv(norm_values_recv)
                        
                        # ---- Reset the state -----
                        worker_data_flag.flag_norm_save = False
                        norm_values_recv = None
                    

            except Exception as e:
                print(f"Here 1: {e}")
                break
        except Exception as e:
            print(f"Here 2: {e}")
            time.sleep(0.01)
            
            
    # --- Grace Exit to delete the children ---
    print("Cleaning up recv_thread...")
    if p1 and p1.is_alive():
        p1.terminate()
        p1.join()
    ser1.close() 
    
#-------------- MiR mode identifier -------------------
FRAME_HEADER_1 = 0xCA
FRAME_HEADER_INPUT = 0xCB
FRAME_HEADER_COMP = 0xCD
#-----------------------------------------------------
    
#-------------- MiR mode (data_10) identifier -------------------
PID_START = 0x10
CONTROL_SHEAR_RATE  = 0x20
#------------------------------------------------------

#-------------- PID mode (data_8) identifier -------------------
PID_LOOP = 0x11
PID_STOP = 0x12 
PID_NORM = 0x13
PID_COEFF_CHANGE = 0x14
#------------------------------------------------------

# ------------ Control Shear Rate (data_8) identifier ----------
CSR_LOOP = 0x21
CSR_COIL_CALIBRATION = 0x22
CSR_NORM = 0x23
CSR_KB = 0x24
# ----------------------------------------------------

def send_thread(serial_data, mode="input", minhall_1=0, maxhall_1=0,minhall_2=0, maxhall_2=0 ) -> None:
    
    if mode == "input":
        worker_combined_send = device_state.TxData()
        combined_send = worker_combined_send.combine_input_data()
        print(combined_send)
        try:
            serial_data.write(combined_send)
        except Exception as e:
            print("Cannot send data!", e)
            

    elif mode == "norm":
        worker_combined_send = device_state.TxData()
        electronic_num = device_state.get_electronics_flag()
        print("electronic_num", electronic_num)
        combined_send = worker_combined_send.combine_additional_data(minhall_1, maxhall_1, minhall_2, maxhall_2,electronic_num)
        print("norm")
        try:
            serial_data.write(combined_send)
        except Exception as e:
            print("Cannot send data!", e)
        
def drain_queue(q):
    """Remove all items from a queue without blocking."""
    while True:
        try:
            q.get_nowait()
        except:
            break
        
        
# ----- Protocol Headers & Formatting ------

# Format: < (Little Endian), f (float), H (unsigned short)
FRAME_SIZE  = BYTES_PER_SAMPLE      # 2 header + 4+4+2+2+4 payload
NORM_SIZE   =  HEADER_SIZE + (4 * (UINT16_SIZE))      # 2 header + 2+2+2+2 payload
KB_SIZE = HEADER_SIZE + (2 * (FLOAT32_SIZE))
FRAME_FMT   = '<ffHHff'  # H1, H2, C1, C2, PD, TORQUE
NORM_FMT    = '<HHHH'   # max_h1, min_h1, max_h2, min_h2
KB_FMT = '<ff' #kb1, kb2

SENSOR_H1 = 0xAA
SENSOR_H2 = 0xAB
NORM_H1   = 0xBA
NORM_H2   = 0xBB

KB_HEADER_1 = 0xBC
KB_HEADER_2 = 0xBD

# ----- Protocol Headers & Formatting ------

def start_process_live_graph(q_to_process, q_to_graph, q_to_csv, q_to_norm, restart_event):
    
    leftover = b''

    while True:
        recv_chunk = q_to_process.get()
        if not recv_chunk:
            continue

        recv_buffer = leftover + recv_chunk
        leftover     = b''
        batch_frames = []

        index      = 0
        buf_len = len(recv_buffer)

        while index < buf_len - 1:

            # ----- NORM frame ------
            if recv_buffer[index] == NORM_H1 and recv_buffer[index+1] == NORM_H2:
                end = index + NORM_SIZE
                if end > buf_len:
                    leftover = recv_buffer[index]
                    break
                norm_data = struct.unpack(NORM_FMT, recv_buffer[index+2 : end])
                q_to_norm.put(norm_data)
                
                print(f"[NORM] Received: {norm_data}")
                
                save_norm_data_to_csv(norm_data)
                
                if batch_frames:
                    q_to_graph.put(batch_frames)
                    q_to_csv.put(batch_frames)
                restart_event.set()
                return
            
            #TODO:for kb
            if recv_buffer[index] == KB_HEADER_1 and recv_buffer[index+1] == KB_HEADER_2:
                end = index + KB_SIZE
                if end > buf_len:
                    leftover = recv_buffer[index:]
                    break
                kb_data = struct.unpack(KB_FMT, recv_buffer[index+2 : end])
                
                print(f"[kb] Received: {kb_data}")
                save_kb_to_csv(kb_data)
                
                if batch_frames:
                    q_to_graph.put(batch_frames)
                    q_to_csv.put(batch_frames)
                    
                restart_event.set()
                return
                
            # ----- Sensor frame -----
            if recv_buffer[index] == SENSOR_H1 and recv_buffer[index+1] == SENSOR_H2:
                end = index + FRAME_SIZE
                if end > buf_len:
                    leftover = recv_buffer[index:]
                    break
                h1, h2, c1, c2, pd, torque = struct.unpack(FRAME_FMT, recv_buffer[index+2 : end])
                batch_frames.extend([h1, h2, c1, c2, pd, torque])
                index = end
                
                continue

            # ── No header match: skip one byte ──
            index += 1

        if batch_frames:
            q_to_graph.put(batch_frames)
            q_to_csv.put(batch_frames)
            
    

def save_sensors_data_to_csv(cleaned_buffer, worker_kb_property,
                worker_specific_downsampling, worker_normalise_properties, num_columns):

    global count_time
    count_time = count_time + 1
    
    file_name = 'dummy.csv'
    
    #------- turn the tuples into numpy arrays -------
    data = np.array(cleaned_buffer)
    
    # Reshape the data to have 'num_columns' columns per row
    reshaped_data = np.array(data).reshape(-1, num_columns)
    

    norm_voltage_1 = reshaped_data[:, 0]          # normalised hall 1 [no unit]
    norm_voltage_2 = reshaped_data[:, 1]             #normalised hall 2 [no unit]
    i1 = reshaped_data[:, 2]           #i1 [digital]
    i2 = reshaped_data[:, 3]             # i2 [digital]
    phase_diff = reshaped_data[:, 4]            # phase diff [rad]
    actual_torque = reshaped_data[:, 5]            # actual torque [rad]
    
    #Current
    i1 = -device_state.change_current_adc(i1)               #convert norm_voltage_1
    i2 = device_state.change_current_adc(i2)               #convert norm_voltage_2

    i1 = device_state.calibration_input_coil_1(i1)
    i2 = device_state.calibration_input_coil_2(i2)

    #----- calibration for  hall sensors -----
    #TODO: Calibration has to be done in MCU too.
    # norm_voltage_1 = device_state.calibrated_hall_sensors1(worker_kb_property.k_b_1,
    #                                                             norm_voltage_1, 
    #                                                             i1 / 1000,
    #                                                             worker_normalise_properties.min_hall_1,
    #                                                             worker_normalise_properties.max_hall_1
    #                                                             )
    # norm_voltage_2 = device_state.calibrated_hall_sensors2(worker_kb_property.k_b_2,
    #                                                             norm_voltage_2, 
    #                                                             i2 / 10000,
    #                                                             worker_normalise_properties.min_hall_2,
    #                                                             worker_normalise_properties.max_hall_2
    #                                                             )
    
    #Average values to reduce amount of data saved
    ####FOR CONSTANT SHEAR RATE 

    ########################################################## debugging purpose ##########################################################
    ########################################################## init object for setter getter ##############################################################################################################
    # #init the object
    # #default tot_average
    # tot_average = worker_specific_downsampling.tot_average
    # print("tot_average:", tot_average)
    # #specified tot_average
    # tot_average_specified = worker_specific_downsampling.tot_average_specified
    # print("tot_average_specified:", tot_average_specified)
    # #default time increment
    # time_increment = worker_specific_downsampling.time_increment
    # print("time_increment:", time_increment)
    # #specified downsampling time increment
    # time_increment_specified = worker_specific_downsampling.time_increment_specified
    # print("time_increment_specified:", time_increment_specified)
    # #current time init
    # current_time = worker_specific_downsampling.current_time
    # print("current_time:", current_time)
    #####################################################################################################################################################################
    
    
    
    #check if the need for specific downsample is needed
    if worker_specific_downsampling.flag_specific_downsample:
            norm_voltage_1 = average_values(norm_voltage_1, worker_specific_downsampling.tot_average_specified).ravel()
            norm_voltage_2 = average_values(norm_voltage_2, worker_specific_downsampling.tot_average_specified).ravel()
            i1 = average_values(i1, worker_specific_downsampling.tot_average_specified).ravel()
            i2 = average_values(i2, worker_specific_downsampling.tot_average_specified).ravel()
            phase_diff = average_values(phase_diff, worker_specific_downsampling.tot_average_specified).ravel()
            actual_torque = average_values(actual_torque, worker_specific_downsampling.tot_average_specified).ravel()
    
    elif worker_specific_downsampling.flag_specific_downsample is False:
            norm_voltage_1 = average_values(norm_voltage_1, worker_specific_downsampling.tot_average).ravel()
            norm_voltage_2 = average_values(norm_voltage_2, worker_specific_downsampling.tot_average).ravel()
            i1 = average_values(i1, worker_specific_downsampling.tot_average).ravel()
            i2 = average_values(i2, worker_specific_downsampling.tot_average).ravel()
            phase_diff = average_values(phase_diff, worker_specific_downsampling.tot_average).ravel()
            actual_torque = average_values(actual_torque, worker_specific_downsampling.tot_average).ravel()

    avg_n = (worker_specific_downsampling.tot_average_specified
             if worker_specific_downsampling.flag_specific_downsample
             else worker_specific_downsampling.tot_average)
    norm_voltage_1, norm_voltage_2, i1, i2, phase_diff, actual_torque = (
        average_values(x, avg_n).ravel()
        for x in (norm_voltage_1, norm_voltage_2, i1, i2, phase_diff, actual_torque)
    )
    
    num_columns_average = 4
    averaged_data = np.zeros((len(norm_voltage_1), num_columns_average))  # shape (100,4)
    averaged_data[:, 0] = norm_voltage_1
    averaged_data[:, 1] = norm_voltage_2
    averaged_data[:, 2] = i1
    averaged_data[:, 3] = i2
    #phase diff
    # averaged_data[:, 4] = phase_diff
    # averaged_data[:, 5] = actual_torque
    
    num_rows = averaged_data.shape[0] 
        
    if worker_specific_downsampling.flag_specific_downsample:
        step = worker_specific_downsampling.time_increment_specified
    else:
        step = worker_specific_downsampling.time_increment

    time_column = (worker_specific_downsampling.current_time + np.arange(num_rows) * step).reshape(-1, 1)

    worker_specific_downsampling.current_time += step * num_rows
    
    final_data = np.hstack((time_column, averaged_data))
    
    #always save the data to file dir
    project_root =  os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    file_name_full = os.path.join(project_root, "files", file_name)
    
    try:
        if not os.path.exists(file_name_full):
            np.savetxt(file_name_full, final_data, header="",  delimiter=";",  comments="", fmt='%.17g')
        else:
            with open(file_name_full, "a") as f:
                np.savetxt(f, final_data, delimiter=";",   fmt='%.17g')
    except Exception as e:
        print(f"The fuck?: {e}")
    
    
#for decreasing data size for csv purposes 
def average_values (col, N):
    """
    Compute the average of every N consecutive elements in a 1D array.

    This function reduces the number of data points by averaging groups of N samples
    from the input array. It is typically used to downsample high-frequency data while
    preserving the overall signal trend.

    Parameters
    ----------
    col : array_like
        1D array of numeric values to be averaged.
        The length of `col` must be a multiple of `N`.
    N : int
        Number of consecutive samples to average together.

    Returns
    -------
    numpy.ndarray
        A 2D column vector (shape: `(len(col) // N, 1)`) containing the averaged values.

    Examples
    --------
    >>> import numpy as np
    >>> col = np.array([1, 2, 3, 4, 5, 6])
    >>> average_values(col, 2)
    array([[1.5],
           [3.5],
           [5.5]])
    """

    average_val = col.reshape(-1, N).mean(axis=1)
    return average_val


def save_norm_data_to_csv(packed_norm_data, filename="normalise_voltage_constant.csv"):
    
    #-------------- turn tuple into numpy array -------------
    data = np.array(packed_norm_data)
    
    #-------------- reshaped into 4 columns ----------------
    data = data.reshape(-1, 4)
    
    headers_name = "min_hall_1_v;max_hall_1_v;min_hall_2_v;max_hall_2_v"
    
    project_root =  os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    file_name_full = os.path.join(project_root, "files", filename)
    
    #-------------- save normalisation variables in csv ----------------
    np.savetxt(file_name_full, data, delimiter=";",fmt='%i', header=headers_name, comments="")
    
    print("Norm vars stored in csv")
    
def save_kb_to_csv(packed_kb_data, filename="k_b_coefficient.csv"):    

            #File path for saving
        header_text = "ELECTRONICS_FLAG;k_b_1;k_b_2"
        project_root =  os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        file_name_full = os.path.join(project_root, "files", filename)
            
        if os.path.exists(file_name_full):
            data = np.genfromtxt(file_name_full, delimiter=";", names=True)
        else:
            #create empty template file if no file exist prior
            data = np.array([], dtype = [
                ("ELECTRONICS_FLAG", "i8"),
                ("k_b_1", "f8"),
                ("k_b_2", "f8")
            ])

        #---- get the current used eletronic flags ----
        electronic_flags = device_state.get_electronics_flag()
        index = data["ELECTRONICS_FLAG"] == (electronic_flags + 1)
        
        if np.any(index):
            data["k_b_1"][index] = packed_kb_data[0]
            data["k_b_2"][index] = packed_kb_data[1]
        else:
            new_row = np.array(
                [(electronic_flags, packed_kb_data[0], packed_kb_data[1])],
                dtype=data.dtype
            )
            data = np.concatenate((data, new_row))

        #---- Save the file again ----
        np.savetxt(
            file_name_full,
            data,
            delimiter=";",
            fmt=["%d", "%.17g", "%.17g"],
            header=header_text,
            comments=""
        )
        
        #---- reload the file ----
        device_state.kbCoefficient.reload()
        
    
if __name__ == "__main__":
    socket_start_connect()
    
    
        