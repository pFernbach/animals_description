#/usr/bin/env python
# Script which goes with animals_description package.
# Easy way to test parabola-planning algo (no internal DoF) on SO3 joint.

from __future__ import division
from hpp.corbaserver.sphere import Robot
from hpp.corbaserver import Client
from hpp.corbaserver import ProblemSolver
from hpp.gepetto import Viewer, PathPlayer
from viewer_display_library import normalizeDir, plotVerticalCone, plotCone, plotPath, plotVerticalConeWaypoints, plotFrame, plotThetaPlane, shootNormPlot, plotStraightLine
from parseLog import parseNodes, parseIntersectionConePlane
import numpy as np
import math

robot = Robot ('sphere')
robot.setJointBounds('base_joint_xyz', [-4, 4, -4, 4, -2, 4])
ps = ProblemSolver (robot)
cl = robot.client
r = Viewer (ps)
pp = PathPlayer (robot.client, r)

# Configs : [x, y, z, rz, ry, rx, dir.x, dir.y, dir.z]
q1 = shootNormPlot (cl, r, 0.5, 0.4, "c1") # init

q2 = shootNormPlot (cl, r, 0.5, 0.4, "c2")
q3 = shootNormPlot (cl, r, 0.5, 0.4, "c3")
q4 = shootNormPlot (cl, r, 0.5, 0.4, "c4")
q5 = shootNormPlot (cl, r, 0.5, 0.4, "c5")
q6 = shootNormPlot (cl, r, 0.5, 0.4, "c6")
q7 = shootNormPlot (cl, r, 0.5, 0.4, "c7")
q8 = shootNormPlot (cl, r, 0.5, 0.4, "c8")
q9 = shootNormPlot (cl, r, 0.5, 0.4, "c9")

plotThetaPlane (q1, q2, r, "ThetaPlane2")
plotThetaPlane (q1, q3, r, "ThetaPlane3")
plotThetaPlane (q1, q4, r, "ThetaPlane4")
plotThetaPlane (q1, q5, r, "ThetaPlane5")
plotThetaPlane (q1, q6, r, "ThetaPlane6")


# Select impact config
q = q3[::]

ps.directPath (q1, q) # directly call the steering method


num_log = 18623
x_vector = []; zPlus_vector = []; zMinus_vector = [];
configs = parseIntersectionConePlane (num_log, x_vector, zPlus_vector, zMinus_vector)
plotStraightLine (vector, q1, r, "inter1")


r( ps.configAtParam(0,0.001) )
ps.pathLength(0)
ps.getWaypoints (0)




## Plot tools ##
plotPath (cl, 0, r, "path", 0.05)

plotVerticalConeWaypoints (cl, 0, r, 0.5, 0.5)

plotFrame (r, "frames", [0,0,3.7], 0.3)


#r.client.gui.addCone ("coneuh", 0.5, 0.7, [0,1,0.3,1]) pb: full cone ....
#r.client.gui.addToGroup ("coneuh", r.sceneName)

# --------------------------------------------------------------------#
## Video capture ##
r.startCapture ("capture","png")
pp(1)
r.stopCapture ()
#ffmpeg -r 50 -i capture_0_%d.png -r 25 -vcodec libx264 video.mp4


# --------------------------------------------------------------------#
## DEBUG commands
cl.obstacle.getObstaclePosition('decor_base')
robot.isConfigValid(q1)
robot.distancesToCollision()
r( ps.configAtParam(0,5) )
ps.optimizePath (0)
ps.clearRoadmap ()
ps.resetGoalConfigs ()
from numpy import *
argmin(robot.distancesToCollision()[0])
robot.getJointNames ()
robot.getConfigSize ()


