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
        - Threads are daemonized where appropriate to allow clean program exit.
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
                        save_to_csv(data_send, worker_kb_property, worker_specific_downsampling)
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

    Notes:
        - The function never terminates on its own (infinite loop).
        - Assumes that each identifier is always followed by at least two bytes.
        - Any bytes not matching the identifier set are ignored.
        - Intended to run in its own process or thread for real-time streaming.

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
    

def save_to_csv(cleaned_buffer, worker_kb_property, worker_specific_downsampling, num_columns=4):
    """
    Process, calibrate, average, and save measurement data to a CSV file.

    This function takes a raw ADC data buffer, reshapes it into the specified number of columns,
    converts raw ADC values into physical quantities (e.g., magnetic field, current), applies
    calibration and averaging, appends a time column, and saves the results to a CSV file.

    The function uses several global parameters and functions for calibration, conversion, and 
    averaging. The CSV file is automatically created in the 'files' directory under the project root.
    If the file already exists, the new data is appended.

    Parameters
    ----------
    cleaned_buffer : array_like
        The input buffer containing ADC samples in a flat structure (1D list or NumPy array).
        Must have a total length that is a multiple of `num_columns`.
    num_columns : int, optional
        Number of columns per data row in `cleaned_buffer`. Default is 4, corresponding to:
        - Column 1: U1 (Hall sensor 1)
        - Column 2: U2 (Hall sensor 2)
        - Column 3: I1 (Current sensor 1)
        - Column 4: I2 (Current sensor 2)

    Global Variables
    ----------------
    file_name : str
        Name of the CSV file where processed data is saved.
    current_time : float
        The current timestamp (in seconds) used to generate the time column.
    tot_average : int
        Averaging factor used to reduce the amount of saved data.
    time_increment : float
        Time difference between successive samples in seconds.
    offset_1, offset_2 : float
        Calibration offsets used in sensor conversion.
    
    Notes
    -----
    - The resulting CSV file will contain columns in the following order:
        1. Time (s)
        2. Calibrated Hall sensor 1 value [U1 / V]
        3. Calibrated Hall sensor 2 value [U2 / V]
        4. Calibrated current coil 1 value [I1 / mA]
        5. Calibrated current coil 2 value  [I2 / mA]

    Raises
    ------
    Exception
        If file writing fails, the error is printed to the console.
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
    
    
    #Justified hall sensors
    col1_converted = packet_transmission.calibrated_hall_sensors1(worker_kb_property.k_b_1, col1_converted, col3_converted/1000)  
    col2_converted = packet_transmission.calibrated_hall_sensors2(worker_kb_property.k_b_2, col2_converted, col4_converted/1000)
    
    
    col3_converted = packet_transmission.calibration_input_coil_1(col3_converted)
    col4_converted = packet_transmission.calibration_input_coil_2(col4_converted)
    
    
    
    #Average values to reduce amount of data saved 
    ####FOR CONSTANT SHEAR RATE 
    
    
    ########################################################## init object for setter getter ##############################################################################################################
    #init the object
    #default tot_average
    tot_average = worker_specific_downsampling.tot_average
    print("tot_average:", tot_average)
    #specified tot_average
    tot_average_specified = worker_specific_downsampling.tot_average_specified
    print("tot_average_specified:", tot_average_specified)
    #default time increment 
    time_increment = worker_specific_downsampling.time_increment
    print("time_increment:", time_increment)
    #specified downsampling time increment
    time_increment_specified = worker_specific_downsampling.time_increment_specified
    print("time_increment_specified:", time_increment_specified)
    #current time init
    current_time = worker_specific_downsampling.current_time
    print("current_time:", current_time)
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
    
    
        