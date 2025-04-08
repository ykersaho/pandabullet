from panda3d.core import *
import pybullet
from direct.showbase.ShowBase import ShowBase
from scipy.spatial.transform import Rotation as R
import trimesh

class Scene(ShowBase):
    def __init__(self, physics):
        ShowBase.__init__(self)
        self.setup_scene()
        self.disableMouse()
        self.taskMgr.add(self.update_display, "update_display",sort=0)
        self.p = physics
        self.objets = []

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

    def add_object(self, name, texturename, mass, scale, type=0,rest=0.9):
        #panda
        model = self.loader.loadModel(name)
        if(texturename != ""):
            texture = self.loader.loadTexture(texturename)  
            model.setTexture(texture, 1)
        model.setScale(scale)
        model.reparentTo(self.render)
        
        #bullet
        # here you can create simple shape or complex shape from model
        match type:
            case 0:  # any
                collision = self.p.createCollisionShape(self.p.GEOM_MESH, fileName=name,meshScale=[scale[0], scale[1], scale[2]])
            case 1:  #sphere
                mesh = trimesh.load(name)
                aabb_min, aabb_max = mesh.bounds
                collision = self.p.createCollisionShape(self.p.GEOM_SPHERE, radius=scale[0]*(aabb_max[0] - aabb_min [0]) / 2)
            case 2:  #box
                mesh = trimesh.load(name)
                aabb_min, aabb_max = mesh.bounds
                half_extents = (aabb_max - aabb_min) / 2
                half_extents[0],half_extents[1],half_extents[2]=scale[0]*half_extents[0],scale[1]*half_extents[1],scale[2]*half_extents[2]
                collision = self.p.createCollisionShape(self.p.GEOM_BOX, halfExtents=half_extents.tolist())

        # body
        collisionid = self.p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=collision)
        self.p.changeDynamics(collisionid, -1, restitution=rest)
        self.objets.append((model,collisionid))
        return model,collisionid

    def update_position(self,mod,col):
        pos,ort = self.p.getBasePositionAndOrientation(col)
        mod.setPos(Vec3(pos[0],pos[1],pos[2]))
        quat = Quat(ort[3],ort[0],ort[1],ort[2])
        mod.setHpr(quat.getHpr())

    def update_display(self,task):
        self.lock.acquire()
        for m,c in self.objets:
            self.update_position(m,c)
        self.lock.release()
        return task.cont
    