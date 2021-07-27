# To execute this program, you need the 'psn' python module. If you are running 
# on Windows, copy the corresponding pre-built module found in the 'libs' folder
# in the script folder.

import psn
import time
import socket

# Helper functions
def get_time_ms(): return int( time.time() * 1000 )
start_time = get_time_ms()
def get_elapsed_time_ms(): return get_time_ms() - start_time

# Encoder / Decoder
encoder = psn.Encoder( "Server 1" )
MULTICAST_TTL = 32
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, MULTICAST_TTL)


multicast_port  = 56565
multicast_group = "236.10.10.10"
# interface_ip    = "10.11.1.43"

# decoder = psn.Decoder();


# Encoder / Decoder
encoder = psn.Encoder( "Server 1" )

# Create two PSN frames with two trackers
for i in range( 0 , 1000 ):
    # Define Trackers
    trackers = {}
    for j in range(1,3):
        tracker = psn.Tracker(j, 'Tracker ' + str(j))
        tracker.set_pos(psn.Float3(j*10+1, j*10+2, j*10+3))
        tracker.set_speed(psn.Float3(j*100+1, j*100+2, j*100+3))
        tracker.set_accel(psn.Float3(j*1000+1, j*1000+2, j*1000+3))
        tracker.set_ori(psn.Float3(j*10000+1, j*10000+2, j*10000+3))
        tracker.set_status(0.1*j)
        tracker.set_target_pos(psn.Float3(j*100000+1, j*100000+2, j*100000+3))
        tracker.set_timestamp(i)
        trackers[tracker.get_id()] = tracker
    
    
    time_stamp = get_elapsed_time_ms()
    packets = []
    packets.extend(encoder.encode_info(trackers, time_stamp))
    packets.extend(encoder.encode_data(trackers, time_stamp))

    
    for packet in packets:
        sock.sendto(packet, (multicast_group, multicast_port))

    time.sleep(1)
