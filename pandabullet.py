from panda3d.core import *
import pybullet as p
import pybullet_data
from direct.showbase.ShowBase import ShowBase
import time
import keyboard
import numpy as np
import threading
from scipy.spatial.transform import Rotation as R
import openvr
import trimesh

# Initialisation du tracker Vive
def get_tracker_pose():
    #get tracker data
    poses = vr_system.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount) 
    for device_idx in range(openvr.k_unMaxTrackedDeviceCount):
        if vr_system.getTrackedDeviceClass(device_idx) == openvr.TrackedDeviceClass_GenericTracker:
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
            return position, rotation.tolist()
    return [0, 0, 0], [0, 0, 0, 1]

def calibrate_tracker():
    position, rotation = get_tracker_pose()
    if position:
        print(f"initial position : {position}")
        print(f"initial rotation : {rotation}")
    return position if position else [0, 0, 0],rotation if rotation else [0, 0, 0]

# tracker Vive initialiation
vr_system = openvr.init(openvr.VRApplication_Scene)

class BouncingBall(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)
        self.setup_tracker()
        self.setup_scene()
        self.setup_world()
        self.lock = Semaphore()
        self.taskMgr.add(self.update_display, "update_display",sort=0)
        threading.Thread(target=self.update_physics).start()

    def setup_tracker(self):
        self.translationoffset,self.rotationoffset = calibrate_tracker()
        self.prev_paddle_pos = np.array([0.0, 0.0, 0.0])
        self.prev_paddle_rot = np.array([0.0, 0.0, 0.0, 1.0])  # Quaternion
        self.prev_time = time.time()

    def setup_scene(self):
        # camera position and light
        self.cam.setPos(0, -30, 4)
        self.cam.lookAt(0, 0, 1)

        # sun light
        sun_light = PointLight('sun_light')
        sun_light.setColor(Vec4(1, 1, 0.9, 1))  # sun color
        sun_light.setShadowCaster(True, 2048, 2048)  # activate shader
        sun_light_np = self.render.attachNewNode(sun_light)
        sun_light_np.setPos(0, 0, 100)  # light position
        self.render.setLight(sun_light_np)

        spot_light = PointLight('spot_light')
        spot_light.setColor(Vec4(1, 1, 0.9, 1))  # sun color
        spot_light_np = self.render.attachNewNode(spot_light)
        spot_light_np.setPos(0, -50, 10)  # light position
        self.render.setLight(spot_light_np)

        # ambiant
        ambient = AmbientLight("ambient")
        ambient.setColor((0.3, 0.3, 0.3, 1))  
        ambient_np = self.render.attachNewNode(ambient)
        self.render.setLight(ambient_np)

        # render shadows
        self.render.setShaderAuto()
        self.autofollow=100

    def add_object(self, name, texturename, mass, scale, type=0,rest=0.9):
        #panda
        model = self.loader.loadModel(name)
        if(texturename != ""):
            texture = self.loader.loadTexture(texturename)  
            model.setTexture(texture, 1)
        model.setScale(scale)
        modelid = model.reparentTo(self.render)
        
        #bullet
        # here you can create simple shape or complex shape from model
        match type:
            case 0:  # any
                collision = p.createCollisionShape(p.GEOM_MESH, fileName=name,meshScale=[scale[0], scale[1], scale[2]])
            case 1:  #sphere
                mesh = trimesh.load(name)
                aabb_min, aabb_max = mesh.bounds
                collision = p.createCollisionShape(p.GEOM_SPHERE, radius=scale[0]*(aabb_max[0] - aabb_min [0]) / 2)
            case 2:  #box
                mesh = trimesh.load(name)
                aabb_min, aabb_max = mesh.bounds
                half_extents = (aabb_max - aabb_min) / 2
                half_extents[0],half_extents[1],half_extents[2]=scale[0]*half_extents[0],scale[1]*half_extents[1],scale[2]*half_extents[2]
                collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents.tolist())

        # body
        collisionid = p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=collision)
        p.changeDynamics(collisionid, -1, restitution=rest)
        return model,collisionid

    def setup_world(self):
        self.playerpos = Vec3(0,-4,0)
        p.connect(p.DIRECT) # use p.GUI if you want to see bullet view
        p.setGravity(0, 0, -9.8)
        p.resetDebugVisualizerCamera(cameraDistance=6, cameraYaw=0, cameraPitch=-30, cameraTargetPosition=[0, 0, 0])
        # Sol physique
        self.groundm, self.groundc = self.add_object("./data/cube.obj", "./data/tennisfull.jpg", 0.0, Vec3(60,60,1))
        q = Quat()
        q.setHpr(Vec3(0,0,180))
        p.resetBasePositionAndOrientation(self.groundc, [0,0,0], q)

        # filet
        self.netm,self.netc = self.add_object("./data/tennisnet.obj", "", 0.0, Vec3(2,2,2),0,0.1)      
        q = Quat()
        q.setHpr(Vec3(90,0,0))
        p.resetBasePositionAndOrientation(self.netc, [0,-1.4,0], q)

        # Balle physique
        self.ballm,self.ballc = self.add_object("./data/ball1.obj", "./data/ball.png", 0.05, Vec3(0.5,0.5,0.5),1)
        p.resetBasePositionAndOrientation(self.ballc, [0,0,3], [0,0,0,1])

        # Raquette physique
        self.racketm,self.racketc = self.add_object("./data/racket.obj", "./data/racket.jpg", 1.0, Vec3(0.05,0.05,0.05),2)
        p.resetBasePositionAndOrientation(self.racketc, [0,0,2], [0,0,0,1])
        
    def update_position(self,mod,col):
        pos,ort = p.getBasePositionAndOrientation(col)
        mod.setPos(Vec3(pos[0],pos[1],pos[2]))
        quat = Quat(ort[3],ort[0],ort[1],ort[2])
        mod.setHpr(quat.getHpr())

    def update_display(self,task):
        self.lock.acquire()
        self.update_position(self.groundm,self.groundc)
        self.update_position(self.netm,self.netc)
        self.update_position(self.ballm,self.ballc)
        self.update_position(self.racketm,self.racketc)
        self.lock.release()
        return task.cont

    def update_physics(self):
        # Simulation
        while True:
            self.lock.acquire()
            # get elapsed time
            curr_time = time.time()
            dt = curr_time - self.prev_time
            # calculate player position
            # evaluate where the ball will touch the ground
            gravity=9.8
            bpos, bort = p.getBasePositionAndOrientation(self.ballc)
            blvel, bavel = p.getBaseVelocity(self.ballc)
            sr= blvel[2]*blvel[2] + 2*gravity*bpos[2]
            if sr>0:
                tf = (-blvel[2] + np.sqrt(sr))/(-gravity)
                if tf < 0:
                    tf = (-blvel[2] - np.sqrt(sr))/(-gravity)
                
                if((blvel[1] < 0) and (blvel[2] < 0)) or (Vec3(blvel).length() < 2):
                    xf = bpos[0] + blvel[0] * tf
                    yf = bpos[1] + blvel[1] * tf
                    self.playerpos.x = self.playerpos.x+(xf-self.playerpos.x)/ self.autofollow
                    self.playerpos.y = self.playerpos.y+(yf-self.playerpos.y)/ self.autofollow
            
            self.cam.setPos(self.playerpos.x, -40, 4)
            self.cam.lookAt(self.playerpos.x, 0, 1)

            # get tracker position and orientation
            position, rotation = get_tracker_pose()
            if position and rotation:
                adjusted_position = [4*(position[i] - self.translationoffset[i]) for i in range(3)]
                adjusted_position[0],adjusted_position[1],adjusted_position[2]=-adjusted_position[0],adjusted_position[2],adjusted_position[1]
                adjusted_position[0],adjusted_position[1],adjusted_position[2] = adjusted_position[0]+self.playerpos.x,adjusted_position[1]+self.playerpos.y,adjusted_position[2]+self.playerpos.z
                adjusted_rotation=rotation
                adjusted_rotation[0],adjusted_rotation[1],adjusted_rotation[2],adjusted_rotation[3]=-rotation[0],rotation[2],rotation[1],rotation[3]
                p.resetBasePositionAndOrientation(self.racketc, adjusted_position, adjusted_rotation)
            else:
                print("No tracker.")
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
            self.prev_time = curr_time

            if position[1] > 1.5:
                p.resetBasePositionAndOrientation(self.ballc, posObj=[0, -30, 3],ornObj=[0, 0, 1, 0])
                self.playerpos = Vec3(0,-30, 0)
                
            if keyboard.is_pressed('c'):
                p.resetBasePositionAndOrientation(self.ballc, posObj=[0, -8, 3],ornObj=[0, 0, 1, 0])
                self.translationoffset,self.rotationoffset = calibrate_tracker()
                self.prev_paddle_pos = np.array([0.0, 0.0, 0.0])
                self.prev_paddle_rot = np.array([0.0, 0.0, 0.0, 1.0])  # Quaternion
                self.prev_time = time.time()
            self.lock.release()
            p.stepSimulation()
            time.sleep(1/240)
        return

if __name__ == "__main__":
    app = BouncingBall()
    app.run()
