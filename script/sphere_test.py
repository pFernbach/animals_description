#/usr/bin/env python
# Script which goes with animals_description package.
# Easy way to test parabola-planning algo (no internal DoF) on SO3 joint.

from hpp.corbaserver.sphere import Robot
from hpp.corbaserver import Client
from hpp.corbaserver import ProblemSolver

robot = Robot ('sphere')
robot.setJointBounds('base_joint_xyz', [-5, 5, -5, 5, -1, 10])
ps = ProblemSolver (robot)
cl = robot.client

# Configs : [x, y, z, rz, ry, rx, dir.x, dir.y, dir.z]
q1 = [-1.5, -1.5, 4.01, 0, 0, 0, 0, 0, 1]; q2 = [1.5, 5.5, 4.01, 0, 0, 0, 0, 0, 1]
r(q2)

from hpp.gepetto import Viewer, PathPlayer
r = Viewer (ps)
pp = PathPlayer (robot.client, r)
r.loadObstacleModel ("animals_description","environment_3d","environment_3d")
r(q1)

ps.setInitialConfig (q1); ps.addGoalConfig (q2); ps.solve ()


ps.pathLength(0)
ps.getWaypoints (0)


## Plot whole path in viewer ##
import numpy as np
nPath = 0
dt = 0.05
for t in np.arange(0., cl.problem.pathLength(nPath), dt):
    lineName = "g"+str(t)
    r.client.gui.addLine(lineName,[cl.problem.configAtParam(nPath, t)[0],cl.problem.configAtParam(nPath, t)[1],cl.problem.configAtParam(nPath, t)[2]],[cl.problem.configAtParam(nPath, t+dt)[0],cl.problem.configAtParam(nPath, t+dt)[1],cl.problem.configAtParam(nPath, t+dt)[2]],[0,0.3,1,1])
    r.client.gui.addToGroup (lineName, r.sceneName)



## Video capture ##
r.startCapture ("capture","png")
pp(1)
r.stopCapture ()
#ffmpeg -r 50 -i capture_0_%d.png -r 25 -vcodec libx264 video.mp4


## DEBUG commands
cl.obstacle.getObstaclePosition('decor_base')
robot.isConfigValid(q1)
robot.distancesToCollision()
r( ps.configAtDistance(0,5) )
ps.optimizePath (0)
ps.clearRoadmap ()
ps.resetGoalConfigs ()
from numpy import *
argmin(robot.distancesToCollision()[0])
robot.getJointNames ()
robot.getConfigSize ()

