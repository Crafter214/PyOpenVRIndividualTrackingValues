#!/usr/bin/env python

import sys
import time
import math
import openvr

vr = openvr.init(openvr.VRApplication_Scene)
poses_t = openvr.TrackedDevicePose_t * openvr.k_unMaxTrackedDeviceCount
poses = []

for i in range(100):
    vrsys = openvr.VRSystem()
    poses, _ = openvr.VRCompositor().waitGetPoses(poses, None)
    hmd_pose = poses[openvr.k_unTrackedDeviceIndex_Hmd]
    #print("hmd_pose: " + str(hmd_pose))

    def convert_to_yaw(pose_mat):
        yaw = 180 / math.pi * math.atan2(pose_mat[1][0], pose_mat[0][0])
        return yaw
    
    def convert_to_pitch(pose_mat):
        pitch = 180 / math.pi * math.atan2(pose_mat[2][0], pose_mat[0][0])
        return pitch

    def convert_to_roll(pose_mat):
        roll = 180 / math.pi * math.atan2(pose_mat[2][1], pose_mat[2][2])
        return roll

    def convert_to_x(pose_mat):
        x = pose_mat[0][3]
        return x

    def convert_to_y(pose_mat):
        y = pose_mat[1][3]
        return y

    def convert_to_z(pose_mat):
        z = pose_mat[2][3]
        return z

    #get controller ids
    def get_controller_ids(vrsys):
        #if vrsys is None:
            #vrsys = openvr.VRSystem()
        #else:
            #vrsys = vrsys
        #left = None
        #right = None
        for i in range(openvr.k_unMaxTrackedDeviceCount):
            device_class = vrsys.getTrackedDeviceClass(i)
            if device_class == openvr.TrackedDeviceClass_Controller:
                role = vrsys.getControllerRoleForTrackedDeviceIndex(i)
                if role == openvr.TrackedControllerRole_RightHand:
                    right = i
                if role == openvr.TrackedControllerRole_LeftHand:
                    left = i
        return left, right

    vrsystem = openvr.VRSystem()

    left_id, right_id = get_controller_ids(vrsystem)
    #print(str(left_id) + " " + str(right_id))
    left_controller_pose = poses[left_id]
    #print("left_c_pose: " + str(left_controller_pose))
    right_controller_pose = poses[right_id]
    #print("right_c_pose: " + str(right_controller_pose))
    
    Yaw = convert_to_yaw(hmd_pose.mDeviceToAbsoluteTracking)
    Pitch = convert_to_pitch(hmd_pose.mDeviceToAbsoluteTracking)
    Roll = convert_to_roll(hmd_pose.mDeviceToAbsoluteTracking)
    X = convert_to_x(hmd_pose.mDeviceToAbsoluteTracking)
    Y = convert_to_y(hmd_pose.mDeviceToAbsoluteTracking)
    Z = convert_to_z(hmd_pose.mDeviceToAbsoluteTracking)

    Left_Controller_Yaw = convert_to_yaw(left_controller_pose.mDeviceToAbsoluteTracking)
    Left_Controller_Pitch = convert_to_pitch(left_controller_pose.mDeviceToAbsoluteTracking)
    Left_Controller_Roll = convert_to_roll(left_controller_pose.mDeviceToAbsoluteTracking)
    Left_Controller_X = convert_to_x(left_controller_pose.mDeviceToAbsoluteTracking)
    Left_Controller_Y = convert_to_y(left_controller_pose.mDeviceToAbsoluteTracking)
    Left_Controller_Z = convert_to_z(left_controller_pose.mDeviceToAbsoluteTracking)

    Right_Controller_Yaw = convert_to_yaw(right_controller_pose.mDeviceToAbsoluteTracking)
    Right_Controller_Pitch = convert_to_pitch(right_controller_pose.mDeviceToAbsoluteTracking)
    Right_Controller_Roll = convert_to_roll(right_controller_pose.mDeviceToAbsoluteTracking)
    Right_Controller_X = convert_to_x(right_controller_pose.mDeviceToAbsoluteTracking)
    Right_Controller_Y = convert_to_y(right_controller_pose.mDeviceToAbsoluteTracking)
    Right_Controller_Z = convert_to_z(right_controller_pose.mDeviceToAbsoluteTracking)
    
    
    #print(left_controller_pose)
    #print(right_controller_pose)
    print("Left Poses: " + str(left_controller_pose.mDeviceToAbsoluteTracking))
    print("Right Poses: " + str(right_controller_pose.mDeviceToAbsoluteTracking))
    print("Left Yaw: " + str(Left_Controller_Yaw))
    print("Right Yaw: " + str(Right_Controller_Yaw))
    print(Yaw)
    sys.stdout.flush()
    time.sleep(0.2)
openvr.shutdown()
