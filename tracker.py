import openvr
import pynput
import ctypes
import numpy as np
import pybullet as p
from panda3d.core import *
import math
from pynput.mouse import Controller, Button

class Tracker:
    def calibrate(self):
        return
    def getTrackerPos(self):
        return [0, 0, 0], [0, 0, 0, 1]

class TrackerVive(Tracker):
    def __init__(self):
        # tracker Vive initialiation
        self.vr_system = openvr.init(openvr.VRApplication_Scene)
        self.calibrate(self)
    def calibrate(self):
        position, rotation = self.getTrackerPos()
        if position:
            print(f"initial position : {position}")
            print(f"initial rotation : {rotation}")
            self.initialPosition = position
            self.initialRotation = rotation
        return
    def getTrackerPos(self):
        #get tracker data
        poses = self.vr_system.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount) 
        for device_idx in range(openvr.k_unMaxTrackedDeviceCount):
            if self.vr_system.getTrackedDeviceClass(device_idx) == openvr.TrackedDeviceClass_GenericTracker:
                pose = poses[device_idx].mDeviceToAbsoluteTracking
                if not hasattr(pose, 'm'):
                    print(f"Tracker {device_idx} : Structure incorrecte")
                    continue            
                matrix = np.array(pose.m)  
                if matrix.shape != (3, 4):  
                    print(f"Tracker {device_idx} : Matrice inattendue {matrix.shape}")
                    continue
                position = matrix[:, 3] # position
                rotation_matrix = matrix[:, :3] # orientation
                rotation = R.from_matrix(rotation_matrix).as_quat()    
                position = [position[i] for i in range(3)]
                adjusted_position = [4*(position[i] - self.initialPosition[i]) for i in range(3)]
                adjusted_position[0],adjusted_position[1],adjusted_position[2]=-adjusted_position[0],adjusted_position[2],adjusted_position[1]
                adjusted_rotation=rotation
                adjusted_rotation[0],adjusted_rotation[1],adjusted_rotation[2],adjusted_rotation[3]=-rotation[0],rotation[2],rotation[1],rotation[3]
                return adjusted_position, adjusted_rotation
        return [0, 0, 0], [0, 0, 0, 1]
    
class TrackerMouse:
    def __init__(self):
        self.mouse = Controller()
    def calibrate(self):
        position = [0, -10, 3]
        rotation = [0, 0, 0, 1]
        position[0] = self.mouse.position[0]
        position[1] = self.mouse.position[1]
        print(f"initial position : {position}")
        print(f"initial rotation : {rotation}")
        self.initialPosition = position
        self.initialRotation = rotation
        self.previousPosition = [0,0,0]
        self.forehand = True
        return
    def getTrackerPos(self):
        position = [0, 0, 3]
        position[0] = (self.mouse.position[0] - self.initialPosition[0])
        position[1] = -(self.mouse.position[1] - self.initialPosition[1])
        dx = position[0] - self.previousPosition[0]
        dy = position[1] - self.previousPosition[1]
        angle = self.previousPosition[2]
        if(dx*dx + dy*dy > 100):
            if(ctypes.windll.user32.GetAsyncKeyState(1) & 0x8000 != 0):
                self.forehand = False
            else:
                self.forehand = True
            self.previousPosition[0] = position[0]
            self.previousPosition[1] = position[1]
            if(self.forehand):
                if(dy<0):
                    angle = math.degrees(math.atan2(dy,dx))-90
                else:
                    angle = 90+math.degrees(math.atan2(dy,dx))
            else:
                if(dx*dy > 0):
                    dx = -math.fabs(dx)
                else:
                    dx = math.fabs(dx)
                dy = math.fabs(dy)
                angle = 90-math.degrees(math.atan2(dy,dx))
            self.previousPosition[2] = angle
        rotation1 = LRotationf(0, 90, 0)  # 90 degrees around the X-axis
        rotation2 = LRotationf(90, 0, 0)  # 90 degrees around the Y-axis        
        rotation3 = LRotationf(0, angle, 0)  # 90 degrees around the Y-axis        
        # Combine rotations
        rotation = rotation3 * rotation2 * rotation1
        x = position[0] / 50
        y = position[1] / 50
        if y < -5:
            z = 0
        else:
            z = (y+5)*(y+5)/10
        return [x,y,z], [rotation[0], rotation[1], rotation[2], rotation[3]]
