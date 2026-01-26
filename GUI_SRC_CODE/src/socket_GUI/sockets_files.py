#functions for socket

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

q_to_process = multiprocessing.Queue()
q_to_graph = multiprocessing.Queue()
q_to_csv = multiprocessing.Queue()


offset_1 = 0
offset_2 = 0


#setter for port_name
port_name = None 

current_time = None
status_connection = None
file_name = ' '


count_time = 0
flag_for_process = None
flag_for_downsampling = None

p1 = None

#for total count receiving from socket (depends if we want 0.5s, 1s or 2s)
tot_count_accumulate_recv = 250




def init_queues():
    global q_to_process, q_to_graph, q_to_csv
    q_to_process = multiprocessing.Queue()
    q_to_graph = multiprocessing.Queue()
    q_to_csv = multiprocessing.Queue()
    print("Multiprocessing queues initialized.")


def port_name_setter(this_port_name):
    global port_name 
    
    port_name = this_port_name


##########################################################################
#start socket connection for USB 
##########################################################################
def socket_start_connect():
    global port_name

    if sys.platform == 'darwin':        #hijazi's laptop
        port_num = '/dev/tty.usbmodem355A357631331'   #for mac1
        # port_name = '/dev/tty.usbmodem355A357631331'       
    elif sys.platform == 'win32':       #simon's laptop
        port_num = port_name
    elif sys.platform == 'linux':
        port_num = '/dev/ttyACM0'

    try:
        ser = serial.Serial(port=port_num, baudrate=baud_rate,timeout=None)
        print(ser)
        print("Connecting to the board")
        print("Successful connection")
        
        status_connection = True
    except Exception as e:
        print("Cannot connect with USB serial port!:", e)
        print(port_name)
        socket_start_connect()  #RECURSIVE TO TRY AGAIN
        
        status_connection = False
        
    return ser

##########################################################################
#start creating two separate thread
##########################################################################
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
    """
    Continuously read incoming data from a serial connection and 
    forward it for live plotting and CSV logging.

    This function runs in an infinite loop, reading fixed-size chunks
    of data from a serial port (`ser1`). The accumulated data is passed
    to a processing pipeline for live visualization and optional saving
    to a CSV file. A separate process is spawned to handle plotting 
    when the first batch of data arrives.

    Args:
        ser1 (serial.Serial):
            An open serial connection object used for reading data.

    Globals:
        flag_for_process (bool):
            Indicates whether the plotting process has been started.
        p1 (multiprocessing.Process):
            Process object for the live plotting subprocess.
        tot_count_accumulate_recv (int):
            Number of read iterations to accumulate into one buffer.

    Workflow:
        1. Initialize an empty buffer (`received_data`).
        2. Accumulate data from the serial port in fixed-size chunks (48 bytes)
           until the count reaches `tot_count_accumulate_recv`.
        3. Once a batch is ready:
            - If the plotting process has not been started yet, spawn it.
            - Forward the raw data to `q_to_process` for unpacking.
        4. Retrieve processed data from `q_to_csv`.
        5. If the runtime flag (`packet_transmission.running_time_flag_getter()`) 
           is set, save the processed data to CSV.

    Error Handling:
        - Any exceptions during serial read or processing are caught and printed.
        - On error, the loop sleeps briefly (0.1s) before retrying.

    Notes:
        - The function is designed to run indefinitely as a background thread.
        - Data is handled in two layers:
            - Raw byte accumulation (`q_to_process` for unpacking).
            - Post-processed integers (`q_to_csv` for logging).
        - The plotting process (`plot_live`) is started only once.

    """
    
    global flag_for_process, p1, tot_count_accumulate_recv, flag_for_downsampling

    worker_data_flag = packet_transmission.RunningTimeFlag()
    worker_process_flag = packet_transmission.ProcessUnpackingFlag()
    worker_normalise_properties = packet_transmission.VoltageNormaliseCoefficient()
    
    #start count with zero
    count = 0
    while True:
        try:
            received_data = b'' #initilaised buffer
            try:
                while count < tot_count_accumulate_recv:
                    count +=1 
                    chunk =  ser1.read(48)
                    received_data += chunk
                count = 0
                
                ## run only once when the program starts
                if not worker_process_flag.flag_process and received_data:
                    p1 = multiprocessing.Process(target=plot_live, args=(q_to_process, q_to_graph, q_to_csv ))
                    p1.start()
                

                    worker_process_flag.flag_process = True
                
                q_to_process.put(obj=received_data) #send to subprocess to be unpacked
                data_send = q_to_csv.get()
                
                # worker_data_flag = packet_transmission.running_time_flag_getter()
                if worker_data_flag.flag_running_time:
                    
                    if data_send:
                        save_to_csv(data_send, worker_kb_property, worker_specific_downsampling, worker_normalise_properties)
                        data_send = None
                    print("Data written!")
            except Exception as e:
                print(f"Here 1: {e}")
        except Exception as e:
            print(f"Here 2: {e}")
            time.sleep(0.01)


##########################################################################
#thread for TCP Tx
##########################################################################
def send_thread(ser1):
    worker_combined_send = packet_transmission.TxData()
    print("here")
    #combined everything
    combined_send = worker_combined_send.combine_data()
    print("Byte send to firmware: ", combined_send)
    #reset the flag
    try:
        ser1.write(combined_send)
    except Exception as e:
        print("Cannot send data!", e)

def plot_live(queue1, q_to_graph, q_to_csv): #q_to_graph to graph (main file)
    start_process_live_graph(queue1, q_to_graph, q_to_csv)

    

def start_process_live_graph(queue1, q_to_graph, q_to_csv):
    """
    Continuously process incoming data for live graphing and CSV logging.

    This function runs in an infinite loop, consuming raw byte streams 
    from `queue1`. It parses the stream by looking for specific identifier 
    bytes and extracting the following two bytes as a 16-bit unsigned integer. 
    The decoded integer values are then forwarded to two queues:
    - `q_to_graph` for live visualization
    - `q_to_csv` for data logging

    Args:
        queue1 (multiprocessing.Queue):
            Input queue containing raw byte streams to process.
        q_to_graph (multiprocessing.Queue):
            Output queue for passing decoded integer data to a graphing process.
        q_to_csv (multiprocessing.Queue):
            Output queue for passing decoded integer data to a CSV writer.

    Processing logic:
        - Identifiers are defined as {b'H', b'I', b'J', b'K'}.
        - When an identifier is found in the stream:
            - The next two bytes are extracted.
            - Interpreted as a little-endian unsigned 16-bit integer (`<H`).
            - Added to the list of decoded values (`tot_chunks`).
        - After processing the full buffer, the collected integers are sent 
          to both output queues.

    """
    

    identifier_bits =  {b'H', b'I', b'J', b'K'}

    while True:
        recv_buffer = queue1.get()
        if recv_buffer:
            tot_chunks = []
            i = 0 
            while i < len(recv_buffer):
                if bytes([recv_buffer[i]]) in identifier_bits:
                    if i + 2 < len(recv_buffer):
                        chunk = recv_buffer[i+1:i+3]    #take the second and third, list slicing works by setting the first and the last array(which it doesnt take)
                        integer_value = struct.unpack('<H', chunk)[0]
                        tot_chunks.append(integer_value)
                    # Skip past the identifier and 2-byte chunk
                    i+= 3
                else:
                    i+=1
            q_to_graph.put(tot_chunks)
            q_to_csv.put(tot_chunks)
            
            
##########################################################################
#write to dummy csv 
##########################################################################     
        
def file_name_change_set(prefix, extension=".csv"):
    
    global file_name
    
    
    file_name = f"{prefix}{extension}"
    

def save_to_csv(cleaned_buffer, worker_kb_property, worker_specific_downsampling, worker_normalise_properties, num_columns=4):
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
        Object containing:
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
    
    count_time = count_time +1
    
    print("How many times has this function been called? :", count_time)

    data = np.array(cleaned_buffer)
    print(len(data))
    # Reshape the data to have 'num_columns' columns per row
    reshaped_data = np.array(data).reshape(-1, num_columns)
    
    
    reshaped_data = reshaped_data.astype(float)
    
    col1 = reshaped_data[:, 0].astype(int)             #take first column (U1)
    col2 = reshaped_data[:, 1].astype(int)                  #take second column (U2)
    col3 = reshaped_data[:, 2].astype(int)                  #take third column (I1)
    col4 = reshaped_data[:, 3].astype(int)                  #take fourth column (I2)

    
    
    #Hall Sensors
    col1_converted = -packet_transmission.change_adc_hall(col1)               #convert col1
    col2_converted = packet_transmission.change_adc_hall(col2)               #convert col2
    
    
    #Current
    col3_converted = -packet_transmission.change_current_adc(col3)               #convert col1
    col4_converted = packet_transmission.change_current_adc(col4)               #convert col2

    col3_converted = packet_transmission.calibration_input_coil_1(col3_converted)
    col4_converted = packet_transmission.calibration_input_coil_2(col4_converted)

    #Justified hall sensors
    col1_converted = packet_transmission.calibrated_hall_sensors1(worker_kb_property.k_b_1, col1_converted, col3_converted/1000)  
    col2_converted = packet_transmission.calibrated_hall_sensors2(worker_kb_property.k_b_2, col2_converted, col4_converted/1000)

    col1_converted = (col1_converted- worker_normalise_properties.zero_offset_voltage_1) / worker_normalise_properties.amplitude_voltage_1
    col2_converted = (col2_converted - worker_normalise_properties.zero_offset_voltage_2) / worker_normalise_properties.amplitude_voltage_2

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
            col1_converted = average_values(col1_converted, worker_specific_downsampling.tot_average_specified).ravel()
            col2_converted = average_values(col2_converted, worker_specific_downsampling.tot_average_specified).ravel()
            col3_converted = average_values(col3_converted, worker_specific_downsampling.tot_average_specified).ravel()
            col4_converted = average_values(col4_converted, worker_specific_downsampling.tot_average_specified).ravel()
    
    elif worker_specific_downsampling.flag_specific_downsample is False:
            col1_converted = average_values(col1_converted, worker_specific_downsampling.tot_average).ravel()
            col2_converted = average_values(col2_converted, worker_specific_downsampling.tot_average).ravel()
            col3_converted = average_values(col3_converted, worker_specific_downsampling.tot_average).ravel()
            col4_converted = average_values(col4_converted, worker_specific_downsampling.tot_average).ravel()

    averaged_data = np.zeros((len(col1_converted), 4))  # shape (100,4)
    averaged_data[:, 0] = col1_converted
    averaged_data[:, 1] = col2_converted
    averaged_data[:, 2] = col3_converted
    averaged_data[:, 3] = col4_converted
    
    num_rows = averaged_data.shape[0] 
        
    if worker_specific_downsampling.flag_specific_downsample:
        step = worker_specific_downsampling.time_increment_specified
    else:
        step = worker_specific_downsampling.time_increment
        
    print("What is the step?", step)

    time_column = (worker_specific_downsampling.current_time + np.arange(num_rows) * step).reshape(-1, 1)

    worker_specific_downsampling.current_time += step * num_rows
    

    final_data = np.hstack((time_column, averaged_data))

            

    #always save the data to file dir
    project_root =  os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    file_name_full = os.path.join(project_root, "files", file_name)
    
    try:
        if not os.path.exists(file_name_full):
            np.savetxt(file_name_full, final_data, header="",  delimiter=";",  comments="", fmt='%.18e')
        else:
            with open(file_name_full, "a") as f:
                np.savetxt(f, final_data, delimiter=";",fmt='%.18e')
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




    
if __name__ == "__main__":
    socket_start_connect()
    
    
        