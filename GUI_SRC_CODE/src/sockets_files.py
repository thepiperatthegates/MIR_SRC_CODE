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
import queue


baud_rate = 128000

append_payload =0 

q_to_process = multiprocessing.Queue()
q_to_graph = multiprocessing.Queue()
q_to_csv = multiprocessing.Queue()

q_get_mir_mode = queue.Queue()
time_increment = None
tot_average = None


offset_1 = 0
offset_2 = 0

current_time = None
file_name = ' '

flag_for_process = None

p1 = None

#for total count receiving from socket (depends if we want 0.5s, 1s or 2s)
tot_count_accumulate_recv = 1250




# q = multiprocessing.Queue()

##########################################################################
#start socket connection for USB 
##########################################################################
def socket_start_connect():
    
    if sys.platform == 'darwin':        #hijazi's laptop
        port_name = '/dev/tty.usbmodem3776345D32331'   #for mac1
        # port_name = '/dev/tty.usbmodem355A357631331'       
    elif sys.platform == 'win32':       #simon's laptop
        port_name = 'COM5'       #for windows 

    try:
        ser = serial.Serial(port=port_name, baudrate=baud_rate,timeout=None)
        print(ser)
        print("Connecting to the board")
        print("Successful connection")
    except Exception as e:
        print("Cannot connect with USB serial port!:", e)
        socket_start_connect()  #RECURSIVE TO TRY AGAIN
        
    return ser

##########################################################################
#start creating two separate thread
##########################################################################
def thread_start():
    global flag_for_process
    ser1 = socket_start_connect()
    #Event for run time receiving data from Serial Porte
    thread_recv = threading.Thread(target=timer_monitor, args=(ser1,))
    thread_recv.start()

    while True:
        start_flag_send = packet_transmission.start_flag_send_getter()
        if start_flag_send == 1:
            packet_transmission.start_flag_send_event(0)
            start_flag_send = 0
            thread_send = threading.Thread(target=send_thread, daemon=True, args=(ser1,))
            thread_send.start()
        time.sleep(0.01)
            

    

def timer_monitor(ser1):
    global flag_for_process, p1, tot_count_accumulate_recv

        
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
                
                
                if not flag_for_process and received_data:
                    p1 = multiprocessing.Process(target=plot_live, args=(q_to_process, q_to_graph, q_to_csv ))
                    p1.start()
                    flag_for_process = True
                q_to_process.put(received_data) #send to subprocess to be unpacked
                
        
                #get unpacked data from queue
                data_Rx = q_to_csv.get()
                
                
                
                data_flag = packet_transmission.running_time_getter()
                
                if data_flag == 1:
                    if data_Rx:
                        print("Done writing once!")
                        save_to_csv(data_Rx)
                        data_Rx = None

                    
            except Exception as e:
                print(f"Here 1: {e}")
        except Exception as e:
            print(f"Here 2: {e}")
            time.sleep(0.1)



def get_offset(get_offset1, get_offset2):
    global offset_1, offset_2
    
    
    offset_1 = get_offset1
    offset_2 = get_offset2
    


##########################################################################
#thread for TCP Tx
##########################################################################
def send_thread(ser1):
    
    
    flag_send = packet_transmission.send_transmission_event_getter()
    if flag_send == 1:
        
        ### Getter function 
        data_send_1 = packet_transmission.data_1_getter()    #for run time
        data_send_7 = packet_transmission.data_7_getter()       #for direction
        data_send_3, data_send_4, data_send_5, data_send_6 = packet_transmission.data_current_start()
        
        #find running frequency
        data_send_2 = packet_transmission.get_frequency_dac()
        
        
        data_send_8 = packet_transmission.get_stop_button_data()
        data_send_9 = packet_transmission.data_hardware_reset_getter()
        data_send_10 = packet_transmission.get_electronic_num()
        
        ### combine the whole data as a sequence byte 
        combined_send = packet_transmission.combine_bytes_for_buffer(data_send_1, data_send_2, data_send_3, data_send_4, 
                                                                    data_send_5, data_send_6, data_send_7, data_send_8, data_send_9, data_send_10)
        
        
        packet_transmission.send_transmission_event(0)
        try:
            ser1.write(combined_send)
        except Exception as e:
            print("Error with socket connection!", e)
    else:
        time.sleep(1)
        
        
        
def plot_live(queue1, q_to_graph, q_to_csv): #q_to_graph to graph (main file)
    start_process_live_graph(queue1, q_to_graph, q_to_csv)

    

def start_process_live_graph(queue1, q_to_graph, q_to_csv):

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
#write to csv
##########################################################################     
        
def file_name_change_set(prefix, extension=".csv"):
    
    global file_name
    
    
    file_name = f"{prefix}{extension}"
    

def save_to_csv(cleaned_buffer, num_columns=4):
    global file_name, current_time, time_increment, tot_average
    global offset_1, offset_2


    data = np.array(cleaned_buffer)
    # Reshape the data to have 'num_columns' columns per row
    reshaped_data = np.array(data).reshape(-1, num_columns)
    
    
    reshaped_data = reshaped_data.astype(float)
    
    col1 = reshaped_data[:, 0]                  #take first column (U1)
    col2 = reshaped_data[:, 1]                  #take second column (U2)
    col3 = reshaped_data[:, 2]                  #take third column (I1)
    col4 = reshaped_data[:, 3]                  #take fourth column (I2)

    #Hall Sensors
    col1_converted = -packet_transmission.change_adc_hall(col1)               #convert col1
    col2_converted = packet_transmission.change_adc_hall(col2)               #convert col2
    
    #Current
    col3_converted = -packet_transmission.change_current_adc(col3)               #convert col1
    col4_converted = packet_transmission.change_current_adc(col4)               #convert col2
    
    #Justified hall sensors
    col1_converted = packet_transmission.calibrated_hall_sensors1(col1_converted, col3_converted/1000)  
    col2_converted = packet_transmission.calibrated_hall_sensors2(col2_converted, col4_converted/1000)
    
    col3_converted = packet_transmission.calibration_input_coil_1(col3_converted)
    col4_converted = packet_transmission.calibration_input_coil_2(col4_converted)
    
    #Average values to reduce amount of data saved 
    
    ####FOR CONSTANT SHEAR RATE 
    col1_after_average = average_values(col1_converted, tot_average).ravel()
    col2_after_average = average_values(col2_converted, tot_average).ravel()
    col3_after_average = average_values(col3_converted, tot_average).ravel()
    col4_after_average = average_values(col4_converted, tot_average).ravel()


    # Stack them into a new array with 100 rows and 4 columns
    averaged_data = np.zeros((len(col1_after_average), 4))  # shape (100,4)
    averaged_data[:, 0] = col1_after_average
    averaged_data[:, 1] = col2_after_average
    averaged_data[:, 2] = col3_after_average
    averaged_data[:, 3] = col4_after_average

    num_rows = averaged_data.shape[0]
    time_column = np.arange(current_time, current_time + (time_increment * num_rows), time_increment).reshape(-1, 1)
    current_time += time_increment * num_rows   

    final_data = np.hstack((time_column, averaged_data))
    

    try:
        if not os.path.exists(file_name):
            np.savetxt(file_name, final_data, header="",  delimiter=";",  comments="", fmt='%.18e')
        else:
            with open(file_name, "a") as f:
                np.savetxt(f, final_data, delimiter=";",fmt='%.18e')
    except Exception as e:
        print("The fuck?: {e}")

        
#for decreasing data size for csv purposes 
def average_values (col, N):

    average_val = col.reshape(-1, N).mean(axis=1).reshape(-1, 1)
    return average_val

    
    
    
    
        