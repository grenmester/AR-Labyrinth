#!/usr/bin/env python

from math import pi, sin, cos
from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from direct.actor.Actor import Actor
from direct.interval.IntervalGlobal import Sequence
from panda3d.core import BitMask32
from panda3d.core import Point3
from panda3d.core import CollisionNode, CollisionBox
import sys

class MyApp(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)

        self.disableMouse()
        self.accept("escape", sys.exit)
        camera.setPosHpr(0, 15, 15, 20, -90, 20)

        # base model
        self.bottom = self.loader.loadModel("models/cube")
        self.bottom.reparentTo(self.render)
        self.bottom.setScale(6, 6, 1)
        self.bottom.setPos(-1, -1, -1)

        # base collision
        self.bottomCollider = self.bottom.attachNewNode(CollisionNode('bottomcnode'))
        self.bottomCollider.node().addSolid(CollisionBox(Point3(0,0,0),Point3(1,1,1)))
        self.bottomCollider.node().setIntoCollideMask(BitMask32.bit(1))
        self.bottomCollider.show()

        for i in range(4):
            for j in range(4):
                exec("self.box" + str(i) + "_" + str(j) + " = self.loader.loadModel('models/cube')")
                exec("self.box" + str(i) + "_" + str(j) + ".reparentTo(self.render)")
                exec("self.box" + str(i) + "_" + str(j) + ".setPos(" + str(i) + ", " + str(j) + ", 0)")

        # ball model
        self.ballRoot = render.attachNewNode("ballRoot")
        self.ball = loader.loadModel("models/ball")
        self.ball.reparentTo(self.ballRoot)
        self.ball.setPos(2,2,2)

        # ball collision
        self.ballSphere = self.ball.find("**/ball")
        self.ballSphere.node().setFromCollideMask(BitMask32.bit(0))
        self.ballSphere.node().setIntoCollideMask(BitMask32.allOff())

        self.taskMgr.add(self.spinCameraTask, "SpinCameraTask")

    def spinCameraTask(self, task):
        angleDegrees = task.time * 20.0
        angleRadians = angleDegrees * (pi / 180.0)
        self.camera.setPos(20 * sin(angleRadians), -20 * cos(angleRadians), 3)
        self.camera.setHpr(angleDegrees, 0, 0)
        #self.camera.setPos(2 * sin(angleRadians), -2 * cos(angleRadians), 25)
        #self.camera.setPos(0,0,25)
        #self.camera.setHpr(task.time*20,-90+task.time*10,task.time*10)
        return Task.cont
        '''
        # Reparent the model to render.
        self.scene.reparentTo(self.render)
        # Apply scale and position transforms on the model.
        self.scene.setScale(0.25, 0.25, 0.25)
        self.scene.setPos(-8, 42, 0)

        # Add the spinCameraTask procedure to the task manager.
        self.taskMgr.add(self.spinCameraTask, "SpinCameraTask")

        # Load and transform the panda actor.
        self.pandaActor = Actor("models/panda-model",
                                {"walk": "models/panda-walk4"})
        self.pandaActor.setScale(0.005, 0.005, 0.005)
        self.pandaActor.reparentTo(self.render)
        # Loop its animation.
        self.pandaActor.loop("walk")

        # Create the four lerp intervals needed for the panda to walk back and forth.
        pandaPosInterval1 = self.pandaActor.posInterval(5,
                                                        Point3(0, -10, 0),
                                                        startPos=Point3(0, 10, 0))
        pandaPosInterval2 = self.pandaActor.posInterval(5,
                                                        Point3(0, 10, 0),
                                                        startPos=Point3(0, -10, 0))
        pandaHprInterval1 = self.pandaActor.hprInterval(2,
                                                        Point3(180, 0, 0),
                                                        startHpr=Point3(0, 0, 0))
        pandaHprInterval2 = self.pandaActor.hprInterval(2,
                                                        Point3(0, 0, 0),
                                                        startHpr=Point3(180, 0, 0))

        # Create and play the sequence that coordinates the intervals.
        self.pandaPace = Sequence(pandaPosInterval1,
                                  pandaHprInterval1,
                                  pandaPosInterval2,
                                  pandaHprInterval2,
                                  name="pandaPace")
        self.pandaPace.loop()

    # Define a procedure to move the camera.
    def spinCameraTask(self, task):
        angleDegrees = task.time * 6.0
        angleRadians = angleDegrees * (pi / 180.0)
        self.camera.setPos(20 * sin(angleRadians), -20.0 * cos(angleRadians), 3)
        self.camera.setHpr(angleDegrees, 0, 0)
        return Task.cont
    '''
app = MyApp()
app.run()
