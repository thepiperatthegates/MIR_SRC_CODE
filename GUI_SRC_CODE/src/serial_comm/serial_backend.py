#functions for socket

from . import device_state as packet_transmission

import numpy as np
import os
import sys
import threading
import time
import multiprocessing
import socket
import struct


# Board's static IP/port (see ZYBO_project/sw/app_component/src/ETH_Server/ETH_Server.h
# -- ETH_IP_ADDR0..3 / ETH_SERVER_PORT -- must match the firmware's lwIP config).
ETH_SERVER_IP   = "192.168.1.10"
ETH_SERVER_PORT = 5001

append_payload =0

q_to_process = multiprocessing.Queue()
q_to_graph = multiprocessing.Queue()
q_to_csv = multiprocessing.Queue()
q_to_watchdog = multiprocessing.Queue()


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
    """Re-initialize all multiprocessing queues (call before restarting the pipeline)."""
    global q_to_process, q_to_graph, q_to_csv, q_to_watchdog
    q_to_process = multiprocessing.Queue()
    q_to_graph = multiprocessing.Queue()
    q_to_csv = multiprocessing.Queue()
    q_to_watchdog = multiprocessing.Queue()
    print("Multiprocessing queues initialized.")


#---------------------- start socket connection over ETH (lwIP TCP server) ----------------------
def socket_start_connect(retries=10, delay=0.5):
    """Open a TCP connection to the board's ETH_Server, retrying up to `retries` times."""
    for attempt in range(retries):
        try:
            sock = socket.create_connection((ETH_SERVER_IP, ETH_SERVER_PORT), timeout=delay * retries)
            sock.settimeout(None)  # blocking recv/sendall once connected, same semantics as pyserial's default
            print(f"Successfully connected to {ETH_SERVER_IP}:{ETH_SERVER_PORT}!")
            return sock
        except OSError as e:
            print(f"Device cannot connect to {ETH_SERVER_IP}:{ETH_SERVER_PORT} ({e}), "
                  f"retrying ({attempt + 1}/{retries}) !........")
            time.sleep(delay)

    raise RuntimeError("Could not connect to device after several attempts")


ETH_RECV_CHUNK_SIZE = 4096

##########################################################################
#start creating two separate thread
##########################################################################
def thread_start():
    """Connect to the device, start the receive thread, and poll for outgoing transmissions."""
    sock = socket_start_connect()
    worker_kb_property = packet_transmission.kbCoefficient()
    worker_specific_downsampling = packet_transmission.DownSampleSpecificFlag()
    worker_normalise_properties = packet_transmission.VoltageNormaliseCoefficient()
    #Event for run time receiving data from the ETH socket
    thread_recv = threading.Thread(target=recv_thread, args=(sock,worker_kb_property, worker_specific_downsampling, worker_normalise_properties))
    thread_recv.start()

    while True:
        packet_transmission.tx_event.wait()
        packet_transmission.tx_event.clear()
        thread_send = threading.Thread(target=send_thread, daemon=False, args=(sock,))
        thread_send.start()

# --- Data Type Sizes (in Byte) ---
UINT8_SIZE   = 1
UINT16_SIZE  = 2 #or float16
FLOAT32_SIZE = 4

# --- Frame Configuration ---
# Frame structure: [Header (2 bytes)] + [Payload]
ADC_BUFFER_SIZE      = 20
HEADER_SIZE          = 2 * UINT8_SIZE
# go back to normal raw data receiving
PAYLOAD_DATA_SIZE    = 4 * UINT16_SIZE
BYTES_PER_SAMPLE     = HEADER_SIZE + PAYLOAD_DATA_SIZE
TOTAL_ONE_CYCLE_BYTES    = BYTES_PER_SAMPLE * ADC_BUFFER_SIZE

SAMPLE_FREQ = 5000  #Hz
SAMPLE_PERIOD = 1.0/(SAMPLE_FREQ)   #s
SAMPLE_PERIOD_TOTAL = SAMPLE_PERIOD * ADC_BUFFER_SIZE   #s
TOT_COUNT_ACCUMULATE_RECV_IN_1_SEC   = int(0.1 / SAMPLE_PERIOD_TOTAL)
# ---------------------- for total count receiving from socket (depends if we want 0.5s, 1s or 2s) ----------------------
TOT_COUNT_ACCUMULATE_RECV_IN_1_SEC_FRONTEND =   int(0.1 / SAMPLE_PERIOD_TOTAL)
range_len = 50000


def recv_thread(sock, worker_kb_property, worker_specific_downsampling, worker_normalise_properties):
    """Accumulate ETH bytes until at least one interval's worth has arrived, forward the batch for live plotting, and save to CSV when recording. """
    global flag_for_process, p1, tot_count_accumulate_recv, flag_for_downsampling
    num_columns = 4
    worker_process_flag = packet_transmission.ProcessUnpackingFlag()
    worker_normalise_properties = packet_transmission.VoltageNormaliseCoefficient()

    carry = b''  # bytes read past target_bytes on the previous batch, carried forward so none get dropped
    while True:
        try:
            try:
                target_bytes = TOT_COUNT_ACCUMULATE_RECV_IN_1_SEC * TOTAL_ONE_CYCLE_BYTES
                received_data = carry
                while len(received_data) < target_bytes:
                    chunk = sock.recv(ETH_RECV_CHUNK_SIZE)
                    if not chunk:
                        raise ConnectionError("ETH_Server closed the connection")
                    received_data += chunk

                received_data, carry = received_data[:target_bytes], received_data[target_bytes:]

                # -----  Start live graph process for the first time --------
                if not worker_process_flag.flag_process:
                    p1 = multiprocessing.Process(target=start_process_live_graph,
                                                  args=(q_to_process, q_to_graph, q_to_csv, q_to_watchdog),
                                                  daemon=True)
                    p1.start()
                    worker_process_flag.flag_process = True
                # ---- Non-blocking check so this loop stays responsive to the socket ----
                sensor_data_recv = None
                try:
                    sensor_data_recv = q_to_csv.get_nowait()
                except Exception:
                    pass  # nothing queued yet

                # ---- Saving data function -------
                if packet_transmission.running_time_event.is_set():
                    if sensor_data_recv:
                        save_to_csv(sensor_data_recv, worker_kb_property, worker_specific_downsampling,
                                    worker_normalise_properties, num_columns=num_columns)
                        sensor_data_recv = None

                q_to_process.put(received_data)
            except (ConnectionError, OSError) as e:
                print(f"[ETH] connection lost ({e}), reconnecting...")
                carry = b''  # bytes from the dead connection, not a continuation of the new one
                time.sleep(1)
                sock = socket_start_connect()
            except Exception as e:
                print(f"Here 1: {e}")
        except Exception as e:
            print(f"Here 2: {e}")
            time.sleep(0.01)


##########################################################################
#thread for TCP Tx
##########################################################################
def send_thread(sock):
    """Pack and transmit the current command data over the ETH socket."""
    worker_combined_send = packet_transmission.TxData()
    #combined everything
    combined_send = worker_combined_send.combine_data()
    #reset the flag
    try:
        sock.sendall(combined_send)
    except Exception as e:
        print("Cannot send data!", e)

        
# ----- Protocol Headers & Formatting ------
# Format: < (Little Endian), f (float), H (unsigned short)
FRAME_SIZE  = BYTES_PER_SAMPLE      # 2 header + 4+4+2+2+4 payload
NORM_SIZE   =  HEADER_SIZE + (4 * (FLOAT32_SIZE))      # 2 header + 2+2+2+2 payload
KB_SIZE = HEADER_SIZE + (2 * (FLOAT32_SIZE))
FRAME_FMT   = '<eeee'  # H1, H2, C1, C2 (uint16_t)
NORM_FMT    = '<ffff'   # max_h1, min_h1, max_h2, min_h2
KB_FMT = '<ff' #kb1, kb2

SENSOR_H1 = 0xAA
SENSOR_H2 = 0xAB
NORM_H1   = 0xBA
NORM_H2   = 0xBB

KB_HEADER_1 = 0xBC
KB_HEADER_2 = 0xBD

WATCHDOG_H1   = 0xCE
WATCHDOG_H2   = 0xCF
WATCHDOG_SIZE = 6
WATCHDOG_FMT  = '<I'

# ----- Protocol Headers & Formatting ------
def start_process_live_graph(q_to_process, q_to_graph, q_to_csv, q_to_watchdog):
    """Parse raw serial bytes into sensor frames and distribute them to the graph and CSV queues."""
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

            # ----- Sensor frame -----
            if recv_buffer[index] == SENSOR_H1 and recv_buffer[index+1] == SENSOR_H2:
                end = index + FRAME_SIZE
                if end > buf_len:
                    leftover = recv_buffer[index:]
                    break
                c1, c2, h1, h2 = struct.unpack(FRAME_FMT, recv_buffer[index+2 : end])
                batch_frames.extend([c1, c2, h1, h2])
                index = end
                continue

            # ----- Watchdog packet -----
            if recv_buffer[index] == WATCHDOG_H1 and recv_buffer[index+1] == WATCHDOG_H2:
                end = index + WATCHDOG_SIZE
                if end > buf_len:
                    leftover = recv_buffer[index:]
                    break
                (seq,) = struct.unpack(WATCHDOG_FMT, recv_buffer[index+2 : end])
                q_to_watchdog.put(seq)
                index = end
                continue

            # ── No header match: skip one byte ──
            index += 1

        if batch_frames:
            q_to_graph.put(batch_frames)
            q_to_csv.put(batch_frames)
            
        
##########################################################################
#write to dummy csv 
##########################################################################     
def file_name_change_set(prefix, extension=".csv"):
    """Set the global output file name used by save_to_csv."""
    global file_name
    
    
    file_name = f"{prefix}{extension}"
    

def save_to_csv(cleaned_buffer, worker_kb_property, worker_specific_downsampling, worker_normalise_properties, num_columns=4):
    """Calibrate, downsample, and append a batch of ADC samples to the CSV file."""

    global file_name, count_time
    
    count_time = count_time +1
    
    print("How many times has this function been called? :", count_time)

    data = np.array(cleaned_buffer)

    # Reshape the data to have 'num_columns' columns per row
    reshaped_data = np.array(data).reshape(-1, num_columns)
    
    
    reshaped_data = reshaped_data.astype(float)
    
    col1 = reshaped_data[:, 0]             #take first column (U1)
    col2 = reshaped_data[:, 1]           #take second column (U2)
    col3 = reshaped_data[:, 2]                #take third column (I1)
    col4 = reshaped_data[:, 3]               #take fourth column (I2)

    
    
    #Hall Sensors
    col1_converted = -packet_transmission.change_adc_hall(col1)               #convert col1
    col2_converted = packet_transmission.change_adc_hall(col2)               #convert col2
    
    
    #Current
    col3_converted = -packet_transmission.change_current_adc(col3)               #convert col1
    col4_converted = packet_transmission.change_current_adc(col4)               #convert col2

    # col3_converted = packet_transmission.calibration_input_coil_1(col3_converted)
    # col4_converted = packet_transmission.calibration_input_coil_2(col4_converted)

    # #Justified hall sensors
    # col1_converted = packet_transmission.calibrated_hall_sensors1(worker_kb_property.k_b_1, col1_converted, col3_converted/1000)  
    # col2_converted = packet_transmission.calibrated_hall_sensors2(worker_kb_property.k_b_2, col2_converted, col4_converted/1000)

    col1_converted = (col1_converted- worker_normalise_properties.zero_offset_voltage_1) / worker_normalise_properties.amp_voltage_1
    col2_converted = (col2_converted - worker_normalise_properties.zero_offset_voltage_2) / worker_normalise_properties.amp_voltage_2

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
            np.savetxt(file_name_full, final_data, header="",  delimiter=";",  comments="", fmt='%.17g')
        else:
            with open(file_name_full, "a") as f:
                np.savetxt(f, final_data, delimiter=";", fmt="%.17g", )
    except Exception as e:
        print(f"The fuck?: {e}")

def downsampling_values(col1, col2, col3, col4, tot_average):
    """Downsample all four sensor columns by averaging every tot_average consecutive samples."""
    col1_after_average = average_values(col1, tot_average).ravel()
    col2_after_average = average_values(col2, tot_average).ravel()
    col3_after_average = average_values(col3, tot_average).ravel()
    col4_after_average = average_values(col4, tot_average).ravel()
    
    return col1_after_average, col2_after_average, col3_after_average, col4_after_average
    
    
#for decreasing data size for csv purposes 
def average_values(col, N):
    """Downsample col by averaging every N consecutive elements. Returns a column vector."""

    average_val = col.reshape(-1, N).mean(axis=1).reshape(-1, 1)
    return average_val




    
if __name__ == "__main__":
    socket_start_connect()
    
    
        