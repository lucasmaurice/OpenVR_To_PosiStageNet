import time
import openvr


openvr.init(openvr.VRApplication_Scene)
poses = []  # will be populated with proper type after first call
while True:
    poses, _ = openvr.VRCompositor().waitGetPoses(poses, None)
    # hmd_pose = poses[openvr.k_unTrackedDeviceIndex_Hmd]
    print("--")
    i = 0
    for k in poses:
        if k.mDeviceToAbsoluteTracking[0][0] != 1:
            print(str(i) + ": " + str(k.mDeviceToAbsoluteTracking[0][0]))
        i += 1

    # print(hmd_pose.mDeviceToAbsoluteTracking)
    time.sleep(1)
openvr.shutdown()
