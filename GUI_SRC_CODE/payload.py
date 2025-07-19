from collections import deque
import numpy as np
import os
from datetime import datetime
import packet_transmission
import struct


current_time = None
file_name = ' '



def file_name_change_set(prefix, extension=".csv"):
    
    global file_name
    
    
    file_name = f"{prefix}{extension}"
    

def save_to_csv(cleaned_buffer, num_columns=4, time_increment=0.0001):
    
    global file_name, current_time

    data = np.array(cleaned_buffer)
    #final_data = cleaned_buffer_array.reshape(-1, number_of_columns)
    
    # try:
    #     if not os.path.exists(file_name):
    #         np.savetxt(file_name, final_data, header="",  delimiter=";",  comments="")
    #         sort_csv()
    #     else:
    #         with open(file_name, "a") as f:
    #             np.savetxt(f, final_data, delimiter=";")
    #             sort_csv()
    # except Exception as e:
    #     print(f"The fuck?: {e}")
        
    #trimmed_size = len(data) - (len(data) % num_columns)
    #data = data[:trimmed_size]  # Trim the extra data at the end
    # trimmed_size = len(data) - (len(data) % num_columns)
    # data = data[:trimmed_size]

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
    col1_converted = packet_transmission.calibrated_hall_sensors(col1_converted, col3_converted/1000)  
    col2_converted = packet_transmission.calibrated_hall_sensors(col2_converted, col4_converted/1000)

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

        
    # headers = np.array([["U1 (V)", "U2 (V)", "I1 (mA)", "I2 (mA)", "Time (s)"]])
    # output_data = np.vstack((headers, final_data.astype(str)))  # Combine headers and data

    try:
        if not os.path.exists(file_name):
            np.savetxt(file_name, final_data, header="",  delimiter=";",  comments="", fmt='%.18e')
        else:
            with open(file_name, "a") as f:
                np.savetxt(f, final_data, delimiter=";",fmt='%.18e')
    except Exception as e:
        print("The fuck?: {e}")

        

