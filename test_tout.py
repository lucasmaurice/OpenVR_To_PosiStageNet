import psn
import time
import socket
import threading
import math
import openvr

pi_2 = math.pi/2

# Helper functions
def get_time_ms(): return int( time.time() * 1000 )
start_time = get_time_ms()
def get_elapsed_time_ms(): return get_time_ms() - start_time

#### This code converts the python hmdmatrix34_t equivalent that you get from pose.mDeviceToAbsoluteTracking into sort-of functioning Euler angles.
#### Euler angles are still Euler angles however, so be careful.

#From https://gist.github.com/awesomebytes/778d4a1720a0ec3af2086ff6a0b057c3
def from_matrix_to_pose_dict(matrix):
    q = {}
    q['w'] = math.sqrt(max(0, 1 + matrix[0][0] + matrix[1][1] + matrix[2][2])) / 2.0
    q['x'] = math.sqrt(max(0, 1 + matrix[0][0] - matrix[1][1] - matrix[2][2])) / 2.0
    q['y'] = math.sqrt(max(0, 1 - matrix[0][0] + matrix[1][1] - matrix[2][2])) / 2.0
    q['z'] = math.sqrt(max(0, 1 - matrix[0][0] - matrix[1][1] + matrix[2][2])) / 2.0
    q['x'] = math.copysign(q['x'], matrix[2][1] - matrix[1][2])
    q['y'] = math.copysign(q['y'], matrix[0][2] - matrix[2][0])
    q['z'] = math.copysign(q['z'], matrix[1][0] - matrix[0][1])
    return q


#Ported from https://www.reddit.com/r/Vive/comments/6toiem/how_to_get_each_axis_rotation_from_vive/dlmczdn/
def NormalizeAngle(angle):
    while (angle > math.pi):
        angle -= math.pi
    while (angle < 0):
        angle += math.pi
    return angle

def NormalizeAngles(angles):
    angles['x'] = NormalizeAngle(angles['x'])
    angles['y'] = NormalizeAngle(angles['y'])
    angles['z'] = NormalizeAngle(angles['z'])
    return angles

def get_proper_euler(matrix):
    in_quat = from_matrix_to_pose_dict(matrix)
    sqw = in_quat['w'] * in_quat['w']
    sqx = in_quat['x'] * in_quat['x']
    sqy = in_quat['y'] * in_quat['y']
    sqz = in_quat['z'] * in_quat['z']
    unit = sqx + sqy + sqz + sqw # if normalised is one, otherwise is correction factor
    test = in_quat['x'] * in_quat['w'] - in_quat['y'] * in_quat['z']

    v = { 'x': 0, 'y': 0, 'z':0 }

    if (test > 0.49995 * unit): #singularity at north pole
        v['y'] = 2.0 * math.atan2(in_quat['y'], in_quat['x'])
        v['x'] = math.pi / 2.0
        v['z'] = 0
        return NormalizeAngles(v)

    if (test > 0.49995 * unit): #singularity at south pole
        v['y'] = -2.0 * math.atan2(in_quat['y'], in_quat['x'])
        v['x'] = -math.pi / 2.0
        v['z'] = 0
        return NormalizeAngles(v)

    v['y'] = math.atan2(2 * in_quat['x'] * in_quat['w'] + 2.0 * in_quat['y'] * in_quat['z'], 1 - 2.0 * (in_quat['z'] * in_quat['z'] + in_quat['w'] * in_quat['w'])) # Yaw
    v['x'] = math.asin(2 * (in_quat['x'] * in_quat['z'] - in_quat['w'] * in_quat['y'])) # Pitch
    v['z'] = math.atan2(2 * in_quat['x'] * in_quat['y'] + 2.0 * in_quat['z'] * in_quat['w'], 1 - 2.0 * (in_quat['y'] * in_quat['y'] + in_quat['z'] * in_quat['z'])) # Roll
    return NormalizeAngles(v)

def convert_to_euler(pose_mat):
    x = pose_mat[0][3]
    y = pose_mat[1][3]
    z = pose_mat[2][3]
    rot = get_proper_euler(pose_mat)
    return [x,y,z,rot['x'],rot['y']-pi_2,rot['z']] #Y is flipped!


# PSN Encoder
encoder = psn.Encoder( "Server 1" )

MULTICAST_TTL = 32
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, MULTICAST_TTL)

multicast_port  = 56565
multicast_group = "236.10.10.10"

encoder = psn.Encoder( "Server 1" )

# INIT OpenVR
openvr.init(openvr.VRApplication_Scene)
poses = []
poses, _ = openvr.VRCompositor().waitGetPoses(poses, None)

# INIT Trackers
trackers = {}
# for i in range(0, len(poses)):
for i in range(0, 5):
    tracker = psn.Tracker(i, 'Tracker ' + str(i))
    trackers[tracker.get_id()] = tracker

# Announce thread
def announce_loop(encoder, trackers):
    while True:
        sock.sendto(encoder.encode_info(trackers, get_elapsed_time_ms())[0], (multicast_group, multicast_port))
        time.sleep(1)

announce_t = threading.Thread(target=announce_loop, args=(encoder,trackers), daemon=True)
announce_t.start()


# Create two PSN frames with two trackers
i = 0
while True:
    i += 1
    for ID in trackers:
        tracker = trackers[ID]
        poses, _ = openvr.VRCompositor().waitGetPoses(poses, None)
        pose = poses[tracker.get_id()].mDeviceToAbsoluteTracking

        [position_x,position_y,position_z,orientation_x,orientation_y,orientation_z] = convert_to_euler(pose)

        tracker.set_pos(psn.Float3(position_x,position_y,position_z))

        # orientation_x = math.sqrt(max(0, 1 + pose[0][0] - pose[1][1] - pose[2][2])) / 2.0
        # orientation_y = math.sqrt(max(0, 1 - pose[0][0] + pose[1][1] - pose[2][2])) / 2.0
        # orientation_z = math.sqrt(max(0, 1 - pose[0][0] - pose[1][1] + pose[2][2])) / 2.0
        # orientation_x = math.copysign(orientation_x, pose[2][1] - pose[1][2])
        # orientation_y = math.copysign(orientation_y, pose[0][2] - pose[2][0])
        # orientation_z = math.copysign(orientation_z, pose[1][0] - pose[0][1])

        
        # orientation_x = pose[0][0]
        # orientation_y = pose[1][1]
        # orientation_z = pose[2][2]
        tracker.set_ori(psn.Float3(orientation_x,orientation_y,orientation_z))

        # print (pose)



    #     tracker = psn.Tracker(j, 'Tracker ' + str(j))
    #     tracker.set_pos(psn.Float3(j*10+1, j*10+2, j*10+3))
    #     tracker.set_speed(psn.Float3(j*100+1, j*100+2, j*100+3))
    #     tracker.set_accel(psn.Float3(j*1000+1, j*1000+2, j*1000+3))
    #     tracker.set_ori(psn.Float3(j*10000+1, j*10000+2, j*10000+3))
    #     tracker.set_status(0.1*j)
    #     tracker.set_target_pos(psn.Float3(j*100000+1, j*100000+2, j*100000+3))
    #     tracker.set_timestamp(i)
    #     trackers[tracker.get_id()] = tracker
    
    
    time_stamp = get_elapsed_time_ms()
    packets = []
    packets.extend(encoder.encode_data(trackers, time_stamp))

    
    for packet in packets:
        sock.sendto(packet, (multicast_group, multicast_port))

    time.sleep(0.1)

