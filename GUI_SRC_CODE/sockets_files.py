#functions for socket

import packet_transmission

import numpy as np
import os

import threading
import time
import multiprocessing
import serial
from live_graphing import plot_live 


port_name = '/dev/tty.usbmodem3776345D32331'   #for mac1
# port_name = '/dev/tty.usbmodem355A357631331'       
    
# port_name = 'COM6'       for windows 
baud_rate = 128000

append_payload =0 

q_to_process = multiprocessing.Queue()
q_to_graph = multiprocessing.Queue()
q_to_csv = multiprocessing.Queue()

current_time = None
file_name = ' '

flag_for_process = None

p1 = None

#for total count receiving from socket (depends if we want 0.5s, 1s or 2s)
tot_count_accumulate_recv = 1250

# q = multiprocessing.Queue()

##########################################################################
#start socket connection for TCP 
##########################################################################
def socket_start_connect():
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
                data_send = q_to_csv.get()
                
                data_flag = packet_transmission.running_time_getter()
                if data_flag == 1:
                    backend_rx_countdown(data_send) #send already-unpacked to be saved in csv
                    
            except Exception as e:
                print(f"Here 1: {e}")
        except Exception as e:
            print(f"Here 2: {e}")
            time.sleep(0.1)

##########################################################################
#write to csv
##########################################################################

def backend_rx_countdown(received_payload):
    if received_payload:
        # print("Processing received_payload:", received_payload)
        # p4 = multiprocessing.Process(target=save_to_csv, args=(received_payload))
        # p4.start()
        # p4.join()
        save_to_csv(received_payload)
        received_payload = None  
    print("Done!")



##########################################################################
#thread for TCP Tx
##########################################################################
def send_thread(ser1):
    flag_send = packet_transmission.send_transmission_event_getter()
    if flag_send == 1:
        data_send_1 = packet_transmission.data_1_getter()    #for run time
        data_send_7 = packet_transmission.data_7_getter()       #for direction
        data_send_3, data_send_4, data_send_5, data_send_6 = packet_transmission.data_current_start()
        
        #find running frequency
        data_send_2 = packet_transmission.get_frequency_dac()
        
        
        data_send_8 = packet_transmission.get_stop_button_data()
        data_send_9 = packet_transmission.data_hardware_reset_getter()
        combined_send = packet_transmission.combine_bytes_for_buffer(data_send_1, data_send_2, data_send_3, data_send_4, 
                                                                     data_send_5, data_send_6, data_send_7, data_send_8, data_send_9)
        packet_transmission.send_transmission_event(0)
        try:
            ser1.write(combined_send)
        except Exception as e:
            print("Error with socket connection!", e)
    else:
        time.sleep(1)
        
        
def file_name_change_set(prefix, extension=".csv"):
    
    global file_name
    
    
    file_name = f"{prefix}{extension}"
    

def save_to_csv(cleaned_buffer, num_columns=4, time_increment=0.0001):
    
    global file_name, current_time

    data = np.array(cleaned_buffer)
    # Reshape the data to have 'num_columns' columns per row
    reshaped_data = np.array(data).reshape(-1, num_columns)
    
    col1 = reshaped_data[:, 0]                  #take first column (U1)
    col2 = reshaped_data[:, 1]                  #take second column (U2)
    col3 = reshaped_data[:, 2]                  #take third column (I1)
    col4 = reshaped_data[:, 3]                  #take fourth column (I2)

    #Hall Sensors
    col1_converted = packet_transmission.change_adc_hall(col1)               #convert col1
    col2_converted = packet_transmission.change_adc_hall(col2)               #convert col2
    
    #Current
    col3_converted = packet_transmission.change_current_adc(col3)               #convert col1
    col4_converted = packet_transmission.change_current_adc(col4)               #convert col2
    
    #Justified hall sensors
    col1_converted = packet_transmission.calibrated_hall_sensors1(col1_converted, col4_converted/1000)  
    col2_converted = packet_transmission.calibrated_hall_sensors2(col2_converted, col3_converted/1000)

    reshaped_data = reshaped_data.astype(float)
    reshaped_data[:, 0] = col1_converted
    reshaped_data[:, 1] = col2_converted
    reshaped_data[:, 2] = col3_converted
    reshaped_data[:, 3] = col4_converted

    num_rows = reshaped_data.shape[0]
    time_column = np.arange(current_time, current_time + (time_increment * num_rows), time_increment).reshape(-1, 1)
    current_time += time_increment * num_rows   

    # Insert the time column as the fifth column
    final_data = np.hstack((time_column, reshaped_data))

    try:
        if not os.path.exists(file_name):
            np.savetxt(file_name, final_data, header="",  delimiter=";",  comments="", fmt='%.18e')
        else:
            with open(file_name, "a") as f:
                np.savetxt(f, final_data, delimiter=";",fmt='%.18e')
    except Exception as e:
        print("The fuck?: {e}")

        
        
            

    
    
    
    
        