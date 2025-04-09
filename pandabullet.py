from panda3d.core import *
import pybullet as p
import scene
from direct.showbase.ShowBase import ShowBase
import time
import keyboard
import numpy as np
import threading
from scipy.spatial.transform import Rotation as R
import trimesh
import tracker
import math
import random

class BouncingBall():
    def __init__(self,scene):
        self.stop=False
        self.reply=False
        self.s = scene
        self.tracker = tracker.TrackerVive()
        self.setup_tracker()
        self.setup_world()
        self.lock = Semaphore()
        scene.lock = self.lock
        threading.Thread(target=self.update_physics).start()

    def setup_tracker(self):
        self.tracker.calibrate()
        self.prev_paddle_pos = np.array([0.0, 0.0, 0.0])
        self.prev_paddle_rot = np.array([0.0, 0.0, 0.0, 1.0])  # Quaternion
        self.prev_time = time.time()

    def setup_world(self):
        self.autofollow=100
        self.playerpos = Vec3(0,-4,0)
        p.connect(p.DIRECT) # use p.GUI if you want to see bullet view
        p.setGravity(0, 0, -9.8)
        p.resetDebugVisualizerCamera(cameraDistance=6, cameraYaw=0, cameraPitch=-30, cameraTargetPosition=[0, 0, 0])
        # Sol physique
        self.groundm, self.groundc = self.s.add_object("./data/cube.obj", "./data/tennisfull.jpg", 0.0, Vec3(60,60,1))
        q = Quat()
        q.setHpr(Vec3(0,0,180))
        p.resetBasePositionAndOrientation(self.groundc, [0,0,0], q)

        # filet
        self.netm,self.netc = self.s.add_object("./data/tennisnet.obj", "", 0.0, Vec3(2,2,2),0,0.1)      
        q = Quat()
        q.setHpr(Vec3(90,0,0))
        p.resetBasePositionAndOrientation(self.netc, [0,-1.8,0], q)

        # Balle physique
        self.ballm,self.ballc = self.s.add_object("./data/ball1.obj", "./data/ball.png", 0.05, Vec3(0.5,0.5,0.5),1)
        p.resetBasePositionAndOrientation(self.ballc, [0,0,3], [0,0,0,1])

        # Raquette physique
        self.racketm,self.racketc = self.s.add_object("./data/racket.obj", "./data/racket.jpg", 1.0, Vec3(0.05,0.05,0.05),2)
        p.resetBasePositionAndOrientation(self.racketc, [0,0,2], [0,0,0,1])
        
    def update_physics(self):
        # Simulation
        while True:
            self.lock.acquire()
            # get elapsed time
            curr_time = time.time()
            dt = curr_time - self.prev_time
            self.prev_time = curr_time
            # calculate player position
            # evaluate where the ball will touch the ground
            gravity=9.8
            d=0.97
            bpos, bort = p.getBasePositionAndOrientation(self.ballc)
            blvel, bavel = p.getBaseVelocity(self.ballc)
            sr= blvel[2]*blvel[2] + 2*gravity*(bpos[2]-0.8)
            if sr>0:
                tf = (blvel[2] + np.sqrt(sr))/(gravity)
                if tf < 0:
                    tf = (blvel[2] - np.sqrt(sr))/(gravity)
                
                xf = bpos[0] + blvel[0] * (1 - math.exp(-d*tf))/d
                yf = bpos[1] + blvel[1] * (1 - math.exp(-d*tf))/d
                if(yf <-50):
                    yf=-50
                if(blvel[1] < 0) or (Vec3(blvel).length() < 2):
                    if(bpos[2] < 1):
                        self.stop=True
                    self.reply=False
                    if(self.stop==False):
                        self.playerpos.x = self.playerpos.x+(xf-self.playerpos.x)/ self.autofollow
                        self.playerpos.y = self.playerpos.y+(yf-self.playerpos.y)/ self.autofollow
                if(blvel[1] > 0 and bpos[1] >0):
                    self.stop=False
                    if(bpos[2] < 1):
                        self.reply=True
                    if(bpos[2] > 2 and self.reply):
                        dx = random.randrange(-20,20)
                        dy = random.randrange(-30,-10)
                        vz = random.randrange(10,20)
                        vx = dx - bpos[0]
                        vy = dy - bpos[1]
                        self.reply=False
                        ball_vel = np.array([vx, vy, vz])
                        angular_vel = np.array([0.0, 0.0, 0.0])
                        p.resetBaseVelocity(self.ballc, ball_vel.tolist(), angular_vel.tolist())

            
            self.s.cam.setPos(self.playerpos.x, -45, 5)
            self.s.cam.lookAt(self.playerpos.x, 0, 1)

            # get tracker position and orientation
            adjusted_position, adjusted_rotation = self.tracker.getTrackerPos()
            adjusted_position[0],adjusted_position[1],adjusted_position[2] = adjusted_position[0]+self.playerpos.x,adjusted_position[1]+self.playerpos.y,adjusted_position[2]+self.playerpos.z
            p.resetBasePositionAndOrientation(self.racketc, adjusted_position, adjusted_rotation)
            if dt > 0:
                # linear speed of the tracker
                paddle_vel = (np.array(adjusted_position) - self.prev_paddle_pos) / dt
                # angular speed of the tracker
                prev_rot_matrix = R.from_quat(self.prev_paddle_rot).as_matrix()
                curr_rot_matrix = R.from_quat(adjusted_rotation).as_matrix()
                rot_diff = curr_rot_matrix @ prev_rot_matrix.T
                angle_axis = R.from_matrix(rot_diff).as_rotvec()
                angular_vel = angle_axis / dt
            else:
                paddle_vel = np.array([0.0, 0.0, 0.0])
                angular_vel = np.array([0.0, 0.0, 0.0])

            # apply velocicty
            p.resetBaseVelocity(self.racketc, paddle_vel.tolist(), angular_vel.tolist())

            # save previous values
            self.prev_paddle_pos = np.array(adjusted_position)
            self.prev_paddle_rot = np.array(adjusted_rotation)

            if adjusted_position[2] > 6:
                p.resetBasePositionAndOrientation(self.ballc, posObj=[0, -30, 3],ornObj=[0, 0, 1, 0])
                self.playerpos = Vec3(0,-30, 0)
                
            if keyboard.is_pressed('c'):
                p.resetBasePositionAndOrientation(self.ballc, posObj=[0, -30, 5],ornObj=[0, 0, 1, 0])
                self.playerpos = Vec3(0,-30, 0)
                self.tracker.calibrate()
                self.prev_paddle_pos = np.array([0.0, 0.0, 0.0])
                self.prev_paddle_rot = np.array([0.0, 0.0, 0.0, 1.0])  # Quaternion
                self.prev_time = time.time()
            self.lock.release()
            p.stepSimulation()
            time.sleep(1/480)
        return

if __name__ == "__main__":
    scene = scene.Scene(p)
    app = BouncingBall(scene)
    scene.run()
