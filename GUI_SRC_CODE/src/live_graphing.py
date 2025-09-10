import struct

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
            
            

