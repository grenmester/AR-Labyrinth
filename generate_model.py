#!/usr/bin/env python

from math import pi, sin, cos
from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from direct.actor.Actor import Actor
from direct.interval.IntervalGlobal import Sequence
from panda3d.core import Material, AmbientLight, DirectionalLight
from panda3d.core import BitMask32, Point3, LVector3
from panda3d.core import CollisionNode, CollisionBox, CollisionRay, CollisionTraverser, CollisionHandlerQueue
import sys

class MyApp(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)

        self.disableMouse()
        self.accept("escape", sys.exit)
        camera.setPosHpr(2, 2, 45, 0, -90, 0)

        # base model
        self.bottom = self.loader.loadModel("models/cube")
        self.bottom.reparentTo(self.render)
        self.bottom.setScale(6, 6, 1)
        self.bottom.setPos(-1, -1, -1)

        # base collision node
        self.bottomCollider = self.bottom.attachNewNode(CollisionNode('bottomcnode'))
        self.bottomCollider.node().addSolid(CollisionBox(Point3(0,0,0),Point3(1,1,1)))
        self.bottomCollider.node().setIntoCollideMask(BitMask32.bit(1))
        self.bottomCollider.show()

        # boundary
        self.grid = [[1,1,1,1],
                     [1,0,0,1],
                     [1,0,0,1],
                     [1,1,1,1]]
        for i in range(len(self.grid)):
            for j in range(len(self.grid[0])):
                if self.grid[i][j]:
                    suffix = str(i) + "_" + str(j)
                    # model
                    exec("self.box" + suffix + " = self.loader.loadModel('models/cube')")
                    exec("self.box" + suffix + ".reparentTo(self.render)")
                    exec("self.box" + suffix + ".setPos(" + str(i) + ", " + str(j) + ", 0)")
                    # collision node
                    exec("self.boxCollider" + suffix + " = self.box" + suffix + ".attachNewNode(CollisionNode('boxcnode" + suffix + "'))")
                    exec("self.boxCollider" + suffix + ".node().addSolid(CollisionBox(Point3(0,0,0),Point3(1,1,1)))")
                    exec("self.boxCollider" + suffix + ".node().setIntoCollideMask(BitMask32.bit(0))")


        # ball model
        self.ballRoot = render.attachNewNode("ballRoot")
        self.ball = loader.loadModel("models/ball")
        self.ball.reparentTo(self.ballRoot)
        self.ball.setPos(2,2,2)

        # ball material
        m = Material()
        m.setSpecular((1, 1, 1, 1))
        m.setShininess(96)
        self.ball.setMaterial(m, 1)

        # ball collision node
        self.ballSphere = self.ball.find("**/ball")
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
        #self.start()
    """
    def spinCameraTask(self, task):
        angleDegrees = task.time * 20.0
        angleRadians = angleDegrees * (pi / 180.0)
        self.camera.setPos(20 * sin(angleRadians), -20 * cos(angleRadians), 3)
        self.camera.setHpr(angleDegrees, 0, 0)
        return Task.cont
    """
    def start(self):
        # The maze model also has a locator in it for where to start the ball
        # To access it we use the find command
        startPos = self.maze.find("**/start").getPos()
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
        norm = colEntry.getSurfaceNormal(render)
        accelSide = norm.cross(LVector3.up())
        # Then that vector is crossed with the surface normal to get a vector that
        # points down the slope. By getting the acceleration in 3D like this rather
        # than in 2D, we reduce the amount of error per-frame, reducing jitter
        self.accelV = norm.cross(accelSide)

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
            self.ballRoot.setPos(newPos)

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
        if base.mouseWatcherNode.hasMouse():
            mpos = base.mouseWatcherNode.getMouse()  # get the mouse position
            self.maze.setP(mpos.getY() * -10)
            self.maze.setR(mpos.getX() * 10)

        # Finally, we move the ball
        # Update the velocity based on acceleration
        self.ballV += self.accelV * dt * ACCEL
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
