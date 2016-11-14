#!/usr/bin/env python

from math import pi, sin, cos
from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from direct.actor.Actor import Actor
from direct.interval.IntervalGlobal import Sequence
from panda3d.core import Material, AmbientLight, DirectionalLight
from panda3d.core import BitMask32, Vec3, Point3, LVector3, LRotationf, Plane
from panda3d.core import CollisionNode, CollisionBox, CollisionSphere, CollisionPlane, CollisionRay, CollisionTraverser, CollisionHandlerQueue
from panda3d.core import WindowProperties
import sys, random
import numpy as np

ACCEL = 70
MAX_SPEED = 5
MAX_SPEED_SQ = MAX_SPEED ** 2

class MyApp(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)

        self.grid = np.loadtxt("input/maze.txt")
        #grid = [[1,1,1,1,1,1],
        #        [1,0,0,0,0,1],
        #        [1,0,0,0,0,1],
        #        [1,0,0,0,0,1],
        #        [1,0,0,0,0,1],
        #        [1,0,0,0,0,1],
        #        [1,1,1,1,1,1]]
        f = open("input/start_point.txt", "r")
        self.start_pos = eval(f.read())

        #wp = WindowProperties()
        #wp.setFullscreen(True)
        #base.win.requestProperties(wp)

        self.disableMouse()
        self.accept("escape", sys.exit)
        self.camera.setPosHpr(self.start_pos[0], self.start_pos[1], 35, 0, -90, 0)
        #self.camera.setPosHpr(0, -20, 4, 0, 0, 0)
        #self.camera.setPosHpr(25, 25, 145, 0, -90, 0)

        # boundary
        offset = 0.05
        for i in range(-1, len(self.grid)+1):
            for j in range(-1, len(self.grid[0])+1):
                if i == -1 or j == -1 or i == len(self.grid) or j == len(self.grid[0]) or self.grid[i][j]:
                    #box-1_-1 is not a valid name so we change it to boxa_b
                    texti = i
                    textj = j
                    if i == -1:
                        texti = 'a'
                    if j == -1:
                        textj = 'b'
                    suffix = str(texti) + "_" + str(textj)
                    # model
                    exec("self.box" + suffix + " = self.loader.loadModel('models/cube')")
                    exec("self.box" + suffix + ".reparentTo(self.render)")
                    exec("self.box" + suffix + ".setPos(" + str(i) + ", " + str(j) + ", 1)")
                    # collision node
                    exec("self.boxCollider" + suffix + " = self.box" + suffix + ".attachNewNode(CollisionNode('wall_collide'))")
                    exec("self.boxCollider" + suffix + ".node().addSolid(CollisionBox(Point3(0-offset,0-offset,0-offset),Point3(1+offset,1+offset,1+offset)))")
                    exec("self.boxCollider" + suffix + ".node().setIntoCollideMask(BitMask32.bit(0))")
                    #exec("self.boxCollider" + suffix + ".show()")

        self.maze = loader.loadModel("models/cube")
        self.maze.setScale(len(self.grid), len(self.grid[0]), 1)
        self.maze.reparentTo(self.render)

        self.walls = self.maze.attachNewNode(CollisionNode('wall_collide'))
        self.walls.node().addSolid(CollisionBox(Point3(0,0,0),Point3(1,1,1)))
        self.walls.node().setIntoCollideMask(BitMask32.bit(1))

        #self.walls.node().setIntoCollideMask(BitMask32.bit(0))

        self.mazeGround = self.maze.attachNewNode(CollisionNode('ground_collide'))
        self.mazeGround.node().addSolid(CollisionPlane(Plane(Vec3(0, 0, 1), Point3(2, 2, 1))))
        self.mazeGround.node().setIntoCollideMask(BitMask32.bit(1))

        # ball model
        self.ballRoot = render.attachNewNode("ballRoot")
        self.ball = loader.loadModel("models/ball")
        self.ball.reparentTo(self.ballRoot)

        # ball material
        m = Material()
        m.setSpecular((1, 1, 1, 1))
        m.setShininess(96)
        self.ball.setMaterial(m, 1)

        # ball collision node
        self.ballSphere = self.ball.find("**/ball")
        #self.ballSphere = self.ball.attachNewNode(CollisionNode('wall_collide'))
        #self.walls.node().addSolid(CollisionSphere(0,0,0,1))
        self.ballSphere.node().setFromCollideMask(BitMask32.bit(0))
        self.ballSphere.node().setIntoCollideMask(BitMask32.allOff())

        # collision ray
        self.ballGroundRay = CollisionRay()
        self.ballGroundRay.setOrigin(0, 0, 10)
        self.ballGroundRay.setDirection(0, 0, -1)

        # ray collision node
        self.ballGroundCol = CollisionNode('groundRay')
        self.ballGroundCol.addSolid(self.ballGroundRay)
        self.ballGroundCol.setFromCollideMask(BitMask32.bit(1))
        self.ballGroundCol.setIntoCollideMask(BitMask32.allOff())

        # ray
        self.ballGroundColNp = self.ballRoot.attachNewNode(self.ballGroundCol)

        self.cTrav = CollisionTraverser()
        self.cHandler = CollisionHandlerQueue()

        self.cTrav.addCollider(self.ballSphere, self.cHandler)
        self.cTrav.addCollider(self.ballGroundColNp, self.cHandler)

        # visual effects
        ambientLight = AmbientLight("ambientLight")
        ambientLight.setColor((.55, .55, .55, 1))
        directionalLight = DirectionalLight("directionalLight")
        directionalLight.setDirection(LVector3(0, 0, -1))
        directionalLight.setColor((0.375, 0.375, 0.375, 1))
        directionalLight.setSpecularColor((1, 1, 1, 1))
        self.ballRoot.setLight(render.attachNewNode(ambientLight))
        self.ballRoot.setLight(render.attachNewNode(directionalLight))

        #self.taskMgr.add(self.spinCameraTask, "SpinCameraTask")
        self.start()

    def spinCameraTask(self, task):
        angleDegrees = task.time * 20.0
        angleRadians = angleDegrees * (pi / 180.0)
        self.camera.setPos(20 * sin(angleRadians), -20 * cos(angleRadians), 3)
        self.camera.setHpr(angleDegrees, 0, 0)
        return Task.cont

    def start(self):
        # The maze model also has a locator in it for where to start the ball
        # To access it we use the find command
        startPos = Point3(self.start_pos[0],self.start_pos[1],2)
        # Set the ball in the starting position
        self.ballRoot.setPos(startPos)
        self.ballV = LVector3(0, 0, 0)         # Initial velocity is 0
        self.accelV = LVector3(0, 0, 0)        # Initial acceleration is 0

        # Create the movement task, but first make sure it is not already
        # running
        taskMgr.remove("rollTask")
        self.mainLoop = taskMgr.add(self.rollTask, "rollTask")

    # This function handles the collision between the ray and the ground
    # Information about the interaction is passed in colEntry
    def groundCollideHandler(self, colEntry):
        # Set the ball to the appropriate Z value for it to be exactly on the
        # ground
        newZ = colEntry.getSurfacePoint(render).getZ()
        self.ballRoot.setZ(newZ + .4)

        # Find the acceleration direction. First the surface normal is crossed with
        # the up vector to get a vector perpendicular to the slope
        #if base.mouseWatcherNode.hasMouse():
        #    mpos = base.mouseWatcherNode.getMouse()

        norm = colEntry.getSurfaceNormal(render)
        #norm = LVector3(mpos.getX(),mpos.getY(),1)
        #accelSide = norm.cross(LVector3.up())
        try:
            f = open("input/acceleration.txt", "r")
            self.accel = eval(f.read())
        except:
            self.accel=(0,0)
        #accelSide = LVector3(random.randint(-1,1),random.randint(-1,1),0)
        accelSide = LVector3(self.accel[0]/60,self.accel[1]/60,0)
        # Then that vector is crossed with the surface normal to get a vector that
        # points down the slope. By getting the acceleration in 3D like this rather
        # than in 2D, we reduce the amount of error per-frame, reducing jitter
        self.accelV = norm.cross(accelSide)
        self.camera.setPosHpr(self.ballRoot.getX(),self.ballRoot.getY(),35, 0, -90, 0)

    # This function handles the collision between the ball and a wall
    def wallCollideHandler(self, colEntry):
        # First we calculate some numbers we need to do a reflection
        norm = colEntry.getSurfaceNormal(render) * -1  # The normal of the wall
        curSpeed = self.ballV.length()                # The current speed
        inVec = self.ballV / curSpeed                 # The direction of travel
        velAngle = norm.dot(inVec)                    # Angle of incidance
        hitDir = colEntry.getSurfacePoint(render) - self.ballRoot.getPos()
        hitDir.normalize()
        # The angle between the ball and the normal
        hitAngle = norm.dot(hitDir)

        # Ignore the collision if the ball is either moving away from the wall
        # already (so that we don't accidentally send it back into the wall)
        # and ignore it if the collision isn't dead-on (to avoid getting caught on
        # corners)
        if velAngle > 0 and hitAngle > .995:
            # Standard reflection equation
            reflectVec = (norm * norm.dot(inVec * -1) * 2) + inVec

            # This makes the velocity half of what it was if the hit was dead-on
            # and nearly exactly what it was if this is a glancing blow
            self.ballV = reflectVec * (curSpeed * (((1 - velAngle) * .5) + .5))
            # Since we have a collision, the ball is already a little bit buried in
            # the wall. This calculates a vector needed to move it so that it is
            # exactly touching the wall
            disp = (colEntry.getSurfacePoint(render) -
                    colEntry.getInteriorPoint(render))
            newPos = self.ballRoot.getPos() + disp
            self.ballRoot.setFluidPos(newPos)

    # This is the task that deals with making everything interactive
    def rollTask(self, task):
        # Standard technique for finding the amount of time since the last
        # frame
        dt = globalClock.getDt()

        # If dt is large, then there has been a # hiccup that could cause the ball
        # to leave the field if this functions runs, so ignore the frame
        if dt > .2:
            return Task.cont

        # The collision handler collects the collisions. We dispatch which function
        # to handle the collision based on the name of what was collided into
        for i in range(self.cHandler.getNumEntries()):
            entry = self.cHandler.getEntry(i)
            name = entry.getIntoNode().getName()
            if name == "wall_collide":
                self.wallCollideHandler(entry)
            elif name == "ground_collide":
                self.groundCollideHandler(entry)
            elif name == "loseTrigger":
                self.loseGame(entry)

        # Read the mouse position and tilt the maze accordingly
        #if base.mouseWatcherNode.hasMouse():
            #mpos = base.mouseWatcherNode.getMouse()  # get the mouse position
            #self.maze.setP(mpos.getY() * -10)
            #self.maze.setR(mpos.getX() * 10)
            #for i in range(len(self.grid)):
            #    for j in range(len(self.grid[0])):
            #        if self.grid[i][j]:
            #            suffix = str(i) + "_" + str(j)
            #            exec("self.box" + suffix + ".setP(mpos.getY() * -10)")
            #            exec("self.box" + suffix + ".setR(mpos.getX() * 10)")
        # Finally, we move the ball
        # Update the velocity based on acceleration
            self.ballV += self.accelV * dt * ACCEL
            #self.ballV = LVector3(mpos.getX()*10,mpos.getY()*10,0)
        # Clamp the velocity to the maximum speed
        if self.ballV.lengthSquared() > MAX_SPEED_SQ:
            self.ballV.normalize()
            self.ballV *= MAX_SPEED
        # Update the position based on the velocity
        self.ballRoot.setPos(self.ballRoot.getPos() + (self.ballV * dt))

        # This block of code rotates the ball. It uses something called a quaternion
        # to rotate the ball around an arbitrary axis. That axis perpendicular to
        # the balls rotation, and the amount has to do with the size of the ball
        # This is multiplied on the previous rotation to incrimentally turn it.
        prevRot = LRotationf(self.ball.getQuat())
        axis = LVector3.up().cross(self.ballV)
        newRot = LRotationf(axis, 45.5 * dt * self.ballV.length())
        self.ball.setQuat(prevRot * newRot)

        return Task.cont       # Continue the task indefinitely

app = MyApp()
app.run()
