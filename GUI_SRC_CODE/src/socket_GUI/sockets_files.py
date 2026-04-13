#---------------------- functions for socket backend ----------------------

import packet_transmission as packet_transmission

import numpy as np
import os
import sys
import threading
import time
import multiprocessing
import serial
import struct


baud_rate = 128000

append_payload =0 

offset_1 = 0
offset_2 = 0


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
file_name = ' '


count_time = 0
flag_for_process = None
flag_for_downsampling = None
flag_test = None

p1 = None

worker_data_flag = packet_transmission.RunningTimeFlag()


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
PAYLOAD_DATA_SIZE    = (2 * UINT16_SIZE) + (3 * FLOAT32_SIZE)
BYTES_PER_SAMPLE     = HEADER_SIZE + PAYLOAD_DATA_SIZE
TOTAL_ONE_CYCLE_BYTES    = BYTES_PER_SAMPLE * DMA_BUFFER_SIZE


# --- Timing & Frequency ---
# Calculations for 10kHz base frequency (0.0001s)
SAMPLE_PERIOD        = 0.0001 * DMA_BUFFER_SIZE
TOT_COUNT_ACCUMULATE_RECV_IN_1_SEC   = int(0.1 / SAMPLE_PERIOD)

# --- Protocol Headers & Formatting ---
# Format: < (Little Endian), f (float), H (unsigned short)
FRAME_SIZE  = BYTES_PER_SAMPLE      # 2 header + 4+4+2+2+4 payload
NORM_SIZE   =  HEADER_SIZE + (4 * (UINT16_SIZE))      # 2 header + 2+2+2+2 payload
FRAME_FMT   = '<ffHHf'  # H1, H2, C1, C2, PD
NORM_FMT    = '<HHHH'   # max_h1, min_h1, max_h2, min_h2

SENSOR_H1 = 0xAA
SENSOR_H2 = 0xAB
NORM_H1   = 0xBA
NORM_H2   = 0xBB

# ---------------------- for total count receiving from socket (depends if we want 0.5s, 1s or 2s) ----------------------
TOT_COUNT_ACCUMULATE_RECV_IN_1_SEC_FRONTEND =   int(0.1 / SAMPLE_PERIOD)



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
    worker_kb_property = packet_transmission.kbCoefficient()
    worker_specific_downsampling = packet_transmission.DownSampleSpecificFlag()
    #Event for run time receiving data from Serial Porte
    thread_recv = threading.Thread(target=recv_thread, args=(ser1,worker_kb_property, worker_specific_downsampling))
    thread_recv.start()
    
    worker_flag_send = packet_transmission.TxFlag()

    while True:
        if worker_flag_send.flag_tx:
            thread_send = threading.Thread(target=send_thread, daemon=False, args=(ser1,))
            thread_send.start()
            #reset the flag
            worker_flag_send.flag_tx = False
        else:
            time.sleep(1)


def recv_thread(ser1, worker_kb_property, worker_specific_downsampling):
    
    #----- pasing worker objects ------------
    worker_process_flag = packet_transmission.ProcessUnpackingFlag()
    worker_normalise_properties = packet_transmission.VoltageNormaliseCoefficient()

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
                                                  worker_normalise_properties)
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
PID_START = 1
CONTROL_SHEAR_RATE  = 2
#------------------------------------------------------

#-------------- PID mode identifier -------------------
PID_LOOP = 0
PID_STOP = 2
PID_NORM = 1
PID_COEFF_CHANGE = 4
#------------------------------------------------------

# ------------ Control Shear Rate identifier ----------
CSR_LOOP = 0
CSR_COIL_CALIBRTAION = 1
CSR_NORM = 2
# ----------------------------------------------------

def send_thread(serial_data):
    worker_combined_send = packet_transmission.TxData()
    combined_send = worker_combined_send.combine_data()
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

            # ----- Sensor frame -----
            if recv_buffer[index] == SENSOR_H1 and recv_buffer[index+1] == SENSOR_H2:
                end = index + FRAME_SIZE
                if end > buf_len:
                    leftover = recv_buffer[index:]
                    break
                h1, h2, c1, c2, pd = struct.unpack(FRAME_FMT, recv_buffer[index+2 : end])
                batch_frames.extend([h1, h2, c1, c2, pd])
                index = end
                
                continue

            # ── No header match: skip one byte ──
            index += 1

        if batch_frames:
            q_to_graph.put(batch_frames)
            q_to_csv.put(batch_frames)
            
            
# ---------------------- write to dummy csv  ----------------------
def file_name_change_set(prefix, extension=".csv"):
    
    global file_name
    
    
    file_name = f"{prefix}{extension}"
    

def save_sensors_data_to_csv(cleaned_buffer, worker_kb_property,
                worker_specific_downsampling, worker_normalise_properties, num_columns=5):
    """
    Process, calibrate, downsample, timestamp, and save measurement data to a CSV file.

    This function takes a raw ADC data buffer, reshapes it into the specified number of
    columns, converts the raw readings into calibrated physical quantities, applies
    optional or default downsampling, generates a time column based on worker-defined
    timing parameters, and appends the resulting data to a CSV file inside the project's
    ``files`` directory.

    Parameters
    ----------
    cleaned_buffer : array_like
        Flat input buffer containing raw ADC samples. Length must be divisible by
        ``num_columns``.
    worker_kb_property : object
        Object providing Hall sensor calibration constants:
            - ``k_b_1`` : calibration coefficient for Hall sensor 1
            - ``k_b_2`` : calibration coefficient for Hall sensor 2
    worker_specific_downsampling : object
        Object controlling downsampling and timing:
            - ``flag_specific_downsample`` : whether user-specified downsampling should be used
            - ``tot_average`` : default averaging factor
            - ``tot_average_specified`` : user-specified averaging factor
            - ``time_increment`` : default sample time step (s)
            - ``time_increment_specified`` : user-specified sample time step (s)
            - ``current_time`` : running timestamp updated after each call
    worker_normalise_properties : object
        Object containing:q_to_norm
            - ``zero_offset_voltage_1`` : normalisation offset for Hall sensor 1
            - ``zero_offset_voltage_2`` : normalisation offset for Hall sensor 2
            - ``amplitude_voltage_1`` : normalisation amplitude for Hall sensor 1
            - ``amplitude_voltage_2`` : normalisation amplitude for Hall sensor 2
    num_columns : int, optional
        Number of columns per row in the ADC buffer. Default is 4:
            1. U1 (Hall sensor 1 / 1st Buffer)
            2. U2 (Hall sensor 2 / 2nd Buffer)
            3. I1 (Current coil 1 / 4th Buffer)
            4. I2 (Current coil 2 / 3rd Buffer)

    Processing Steps
    ----------------
    1. Reshape buffer into rows of ``num_columns``.
    2. Convert raw ADC readings:
        - Hall sensors using ``packet_transmission.change_adc_hall``
        - Current coils using ``packet_transmission.change_current_adc`` and coil-specific calibration
    3. Apply Hall sensor justification using ``k_b_1`` and ``k_b_2``.
    4. Apply normalisation using zero-offset and amplitude-voltage parameters.
    5. Apply downsampling:
        - If ``flag_specific_downsample`` is True → use ``tot_average_specified``.
        - Otherwise → use ``tot_average``.
    6. Construct a time column using:
        - ``current_time`` as the starting time
        - ``time_increment`` or ``time_increment_specified`` as the step
    7. Update ``current_time`` inside ``worker_specific_downsampling``.
    8. Save result as semicolon-separated CSV:
        - If file does not exist → create it.
        - If it exists → append new rows.

    Output Format
    -------------
    The saved CSV contains the following columns:
        1. Time / s
        2. Normalised Hall sensor 1 value
        3. Normalised Hall sensor 2 value
        4. Calibrated current coil 1 / mA
        5. Calibrated current coil 2 / mA

    File Location
    -------------
    The CSV file is always saved under:
        ``<project_root>/files/<file_name>``
    """

    global file_name, count_time
    
    count_time = count_time + 1
    
    #------- turn the tuples into numpy arrays -------
    data = np.array(cleaned_buffer)
    
    # Reshape the data to have 'num_columns' columns per row
    reshaped_data = np.array(data).reshape(-1, num_columns)
    

    col1 = reshaped_data[:, 0]          # normalised hall 1 [no unit]
    col2 = reshaped_data[:, 1]             #normalised hall 2 [no unit]
    col3 = reshaped_data[:, 2]           #i1 [digital]
    col4 = reshaped_data[:, 3]             # i2 [digital]
    col5 = reshaped_data[:, 4]            # phase diff [rad]
    
    #Current
    col3 = -packet_transmission.change_current_adc(col3)               #convert col1
    col4 = packet_transmission.change_current_adc(col4)               #convert col2

    col3 = packet_transmission.calibration_input_coil_1(col3)
    col4 = packet_transmission.calibration_input_coil_2(col4)

    #Justified hall sensors
    col1 = packet_transmission.calibrated_hall_sensors1(worker_kb_property.k_b_1, col1, col3/1000)  
    col2 = packet_transmission.calibrated_hall_sensors2(worker_kb_property.k_b_2, col2, col4/1000)

    # col1 = (col1- worker_normalise_properties.zero_offset_voltage_1) / worker_normalise_properties.amp_voltage_1
    # col2 = (col2 - worker_normalise_properties.zero_offset_voltage_2) / worker_normalise_properties.amp_voltage_2

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
            col1 = average_values(col1, worker_specific_downsampling.tot_average_specified).ravel()
            col2 = average_values(col2, worker_specific_downsampling.tot_average_specified).ravel()
            col3 = average_values(col3, worker_specific_downsampling.tot_average_specified).ravel()
            col4 = average_values(col4, worker_specific_downsampling.tot_average_specified).ravel()
            col5 = average_values(col5, worker_specific_downsampling.tot_average_specified).ravel()
    
    elif worker_specific_downsampling.flag_specific_downsample is False:
            col1 = average_values(col1, worker_specific_downsampling.tot_average).ravel()
            col2 = average_values(col2, worker_specific_downsampling.tot_average).ravel()
            col3 = average_values(col3, worker_specific_downsampling.tot_average).ravel()
            col4 = average_values(col4, worker_specific_downsampling.tot_average).ravel()
            col5 = average_values(col5, worker_specific_downsampling.tot_average).ravel()

    averaged_data = np.zeros((len(col1), num_columns))  # shape (100,4)
    averaged_data[:, 0] = col1
    averaged_data[:, 1] = col2
    averaged_data[:, 2] = col3
    averaged_data[:, 3] = col4
    averaged_data[:, 4] = col5
    
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

def downsampling_values (col1, col2, col3, col4, tot_average):
        
    col1_after_average = average_values(col1, tot_average).ravel()
    col2_after_average = average_values(col2, tot_average).ravel()
    col3_after_average = average_values(col3, tot_average).ravel()
    col4_after_average = average_values(col4, tot_average).ravel()
    
    return col1_after_average, col2_after_average, col3_after_average, col4_after_average
    
    
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

    average_val = col.reshape(-1, N).mean(axis=1).reshape(-1, 1)
    return average_val


def save_norm_data_to_csv(packed_norm_data, filename="normalise_voltage_constant.csv"):
    
    #-------------- turn tuple into numpy array -------------
    data = np.array(packed_norm_data)
    
    #-------------- reshaped into 4 columns ----------------
    data = data.reshape(-1, 4)
    
    headers_name = "max_hall_1_V;zero_offset_hall_1_V;max_hall_2_V;zero_offset_hall_2_V"
    
    project_root =  os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    file_name_full = os.path.join(project_root, "files", filename)
    
    #-------------- save normalisation variables in csv ----------------
    np.savetxt(file_name_full, data, delimiter=";",fmt='%i', header=headers_name, comments="")
    
    print("Norm vars stored in csv")
    
if __name__ == "__main__":
    socket_start_connect()
    
    
        