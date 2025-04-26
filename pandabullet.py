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
import player

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
        self.player1=player.Player(1)
        self.player2=player.Player(-1)
        self.trajectorypos = [[0, 0, 5]]*1000
        self.trajectoryrot = [[0, 0, 0, 1]]*1000
        threading.Thread(target=self.update_physics).start()

    def setup_tracker(self):
        self.tracker.calibrate()
        self.prev_paddle_pos = np.array([0.0, 0.0, 0.0])
        self.prev_paddle_rot = np.array([0.0, 0.0, 0.0, 1.0])  # Quaternion
        self.prev_time = time.time()

    def setup_world(self):
        self.autofollow=100
        p.connect(p.DIRECT) # use p.GUI if you want to see bullet view
        p.setGravity(0, 0, -9.8)
        p.resetDebugVisualizerCamera(cameraDistance=6, cameraYaw=0, cameraPitch=-30, cameraTargetPosition=[0, 0, 0])
        # Sol physique
        self.groundm, self.groundc = self.s.add_object("./data/cube.obj", "./data/tennisfull.jpg", 0.0, Vec3(60,60,0.3))
        q = Quat()
        q.setHpr(Vec3(0,0,180))
        p.resetBasePositionAndOrientation(self.groundc, [0,0,0], q)

        # filet
        self.netm,self.netc = self.s.add_object("./data/tennisnet.obj", "", 0.0, Vec3(2,2,2),0,0.1)      
        q = Quat()
        q.setHpr(Vec3(90,0,0))
        p.resetBasePositionAndOrientation(self.netc, [0,-1.2,0], q)

        # Balle physique
        self.ballm,self.ballc = self.s.add_object("./data/ball1.obj", "./data/ball.png", 0.05, Vec3(0.5,0.5,0.5),1)
        p.resetBasePositionAndOrientation(self.ballc, [0,0,3], [0,0,0,1])

        # Raquette physique
        self.racketm,self.racketc = self.s.add_object("./data/racket.obj", "./data/racket.jpg", 1.0, Vec3(0.05,0.05,0.05),2)
        p.resetBasePositionAndOrientation(self.racketc, [0,0,2], [0,0,0,1])

        # Raquette physique
        self.racket2m,self.racket2c = self.s.add_object("./data/racket.obj", "./data/racket.jpg", 0.0, Vec3(0.05,0.05,0.05),2)
        p.resetBasePositionAndOrientation(self.racketc, [0,0,2], [0,0,0,1])

    def update_physics(self):
        # Simulation
        while True:
            self.lock.acquire()
            # get elapsed time
            curr_time = time.time()
            dt = curr_time - self.prev_time
            self.prev_time = curr_time
            # update player position
            gravity=9.8
            d=0.97
            bpos, bort = p.getBasePositionAndOrientation(self.ballc)
            blvel, bavel = p.getBaseVelocity(self.ballc)
            playerpos1 = self.player1.updateposition(bpos,blvel)
            playerpos2 = self.player2.updateposition(bpos,blvel)
            if(bpos[2] > 2 and self.player2.getreply() and blvel[2] < 0):
                x1 = random.randrange(-20,20)
                y1 = random.randrange(-30,-25)
                vz = random.randrange(15,18)
                sr= vz*vz + 2*gravity*(vz-0.8)
                if sr>0:
                    tf = (vz + np.sqrt(sr))/(gravity)
                    if tf < 0:
                        tf = (vz - np.sqrt(sr))/(gravity)                
                vx = (x1 - bpos[0])*d/(1-math.exp(-d*tf))
                vy = (y1 - bpos[1])*d/(1-math.exp(-d*tf))
                self.reply=False
                ball_vel = np.array([vx, vy, vz])
                angular_vel = np.array([0.0, 0.0, 0.0])
                p.resetBaseVelocity(self.ballc, ball_vel.tolist(), angular_vel.tolist())
            
            self.s.cam.setPos(playerpos1[0], playerpos1[1] - 15, 5)
            self.s.cam.lookAt(playerpos1[0], playerpos1[1] + 30, 1)

            # get tracker position and orientation
            adjusted_position, adjusted_rotation = self.tracker.getTrackerPos()
            index = int(abs(bpos[1])*100/30)
            if(blvel[1]*bpos[1] < 0):
                index = index + 500
            if(bpos[1] < 0):
                if(index > 0 and index < len(self.trajectoryrot)):
                    self.trajectoryrot[index] = [adjusted_rotation[0],adjusted_rotation[1],adjusted_rotation[2],adjusted_rotation[3]]
                    self.trajectorypos[index] = [adjusted_position[0],adjusted_position[1],adjusted_position[2]]
            else:
                if(index > 0 and index < len(self.trajectoryrot)):
                    p_adjusted_rotation=self.trajectoryrot[index]
                    adjusted_rotation2 = Quat(p_adjusted_rotation[0],p_adjusted_rotation[1],-p_adjusted_rotation[2],p_adjusted_rotation[3])
                    p_adjusted_position=self.trajectorypos[index]
                    adjusted_position2 = [p_adjusted_position[0]+playerpos2[0],p_adjusted_position[1]+playerpos2[1],p_adjusted_position[2]+playerpos2[2]]
                    p.resetBasePositionAndOrientation(self.racket2c, adjusted_position2, adjusted_rotation2)
            
            adjusted_position[0],adjusted_position[1],adjusted_position[2] = adjusted_position[0]+playerpos1[0],adjusted_position[1]+playerpos1[1],adjusted_position[2]+playerpos1[2]
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
                self.player1.setposition([0,-30, 0])
                self.player2.setposition([0,30, 0])
                
            if keyboard.is_pressed('c'):
                p.resetBasePositionAndOrientation(self.ballc, posObj=[0, -30, 5],ornObj=[0, 0, 1, 0])
                self.player1.setposition([0,-30, 0])
                self.player2.setposition([0,30, 0])
                self.tracker.calibrate()
                self.prev_paddle_pos = np.array([0.0, 0.0, 0.0])
                self.prev_paddle_rot = np.array([0.0, 0.0, 0.0, 1.0])  # Quaternion
                self.prev_time = time.time()
            p.stepSimulation()
            self.lock.release()
            time.sleep(1/480)
        return

if __name__ == "__main__":
    scene = scene.Scene(p)
    app = BouncingBall(scene)
    scene.run()
