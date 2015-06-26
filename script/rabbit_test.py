#/usr/bin/env python
# Script which goes with animals_description package.
# Easy way to test planning algo (no internal DoF) on SO3 joint.

from hpp.corbaserver.rabbit import Robot
from hpp.corbaserver import Client
from hpp.corbaserver import ProblemSolver
import time
import numpy as np

robot = Robot ('rabbit')
robot.setJointBounds('base_joint_xyz', [-5, 5, -5, 5, -0.01, 3])
ps = ProblemSolver (robot)
cl = robot.client

# Configs : [x, y, z, rz, ry, rx, (gamma)]
q1 = [-0.65, -0.3, 0.1, 0, 0, 0, 0]; q2 = [1.55, 2.5, 0.123, 1.57, 0, 0, 0]

from hpp.gepetto import Viewer, PathPlayer
r = Viewer (ps)
pp = PathPlayer (robot.client, r)
r.loadObstacleModel ("animals_description","waterLily_envir","waterLily")
r(q1)

ps.setInitialConfig (q1); ps.addGoalConfig (q2)

# Load box obstacle in HPP for collision avoidance
#cl.obstacle.loadObstacleModel('puzzle_description','decor_very_easy','')

ps.solve ()
ps.pathLength(0)
ps.getWaypoints (0)



r.startCapture ("capture","png")
pp(1)
r.stopCapture ()
#ffmpeg -r 50 -i capture_0_%d.png -r 25 -vcodec libx264 video.mp4


## DEBUG commands
cl.obstacle.getObstaclePosition('decor_base')
robot.getJointOuterObjects('j_object_one')
robot.getJointOuterObjects('j_object_two')
robot.getJointOuterObjects('j_object_three')
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

