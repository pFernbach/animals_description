#/usr/bin/env python
# Script which goes with animals_description package.
# Easy way to test planning algo (no internal DoF) on SO3 joint.

from hpp.corbaserver.ant_sphere import Robot
from hpp.corbaserver import Client
from hpp.corbaserver import ProblemSolver
import numpy as np
from viewer_display_library import normalizeDir, plotVerticalCone, plotCone, plotPath, plotVerticalConeWaypoints, plotFrame, plotThetaPlane, shootNormPlot, plotStraightLine, plotConeWaypoints
from parseLog import parseNodes, parseIntersectionConePlane, parseAlphaAngles

robot = Robot ('robot')
#robot.setJointBounds('base_joint_xyz', [-2, 2, -2, 2, -0.1, 8])
robot.setJointBounds('base_joint_xyz', [-6, 6, -6, 6, -2, 8])
ps = ProblemSolver (robot)
cl = robot.client

# Configs : [x, y, z, qw, qx, qy, qz, nx, ny, nz, theta]
#q1 = [-0.015348906975285266, -4.171305044110657, -0.9392091187458651, 0.9315152701322247, 0.1552333289221375, 0.03148397146866394, -0.32739986964383516, -0.0429911429320373, -0.30982012895828037, 0.9498227462646237, 0]
#q2 = [-0.43304741448409084, -4.349158016911722, 0.9838203122978445, -0.11403063817687004, 0.59745220847309, 0.7873253003678657, -0.10083027099166687, -0.30004094907731027, -0.022516733564641164, -0.9536605400174454, 0]
q1 = [0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0]
q2 = [0.3, 0, 3, 1, 0, 0, 0, 0, 0, -1, 0]

from hpp.gepetto import Viewer, PathPlayer
r = Viewer (ps)
pp = PathPlayer (robot.client, r)
#r.loadObstacleModel ("animals_description","cave","cave")
#cl.obstacle.loadObstacleModel('animals_description','cave','cave')
r(q2)

ps.setInitialConfig (q1); ps.addGoalConfig (q2); ps.solve ()

# verif orientation:
r(ps.getWaypoints (0)[0]) # ref
r(ps.getWaypoints (1)[0]) # should be good...

index = cl.robot.getConfigSize () - 4
q = q1[::]
plotStraightLine ([q[index], q[index+1], q[index+2]], q, r, "normale")
plotCone (q1, cl, r, 0.5, 0.2, "cone1")
plotCone (q2, cl, r, 0.5, 0.2, "cone2")

r( ps.configAtParam(0,2) )
ps.pathLength(0)
ps.getWaypoints (0)
ps.getWaypoints (1)

plotPath (cl, 0, r, "pathy", 0.1) # time-step should depend on sub-path length ?

plotFrame (r, "framy", [0,0,0], 0.5)
plotThetaPlane (q1, q2, r, "ThetaPlane")



## DEBUG commands
cl.obstacle.getObstaclePosition('decor_base')
robot.getJointOuterObjects('base_joint_xyz')
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


 texcoord="UVMap"
<instance_material symbol="stone14-material" target="#stone14-material">
  <bind_vertex_input semantic="UVMap" input_semantic="TEXCOORD" input_set="0"/>
</instance_material>


tanTheta = (q2 [1] - q1 [1]) / (q2 [0] - q1 [0])
num_log = 10711
line = 522 # "q: ..." line
configs, xPlus_vector, xMinus_vector, zPlus_vector, zMinus_vector = parseIntersectionConePlane (num_log, str(line)+': q: ', str(line+1)+': x_plus: ', str(line+2)+': x_minus: ', str(line+3)+': z_x_plus: ', str(line+4)+': z_x_minus: ')
i = 0
plotStraightLine ([xPlus_vector[i], xPlus_vector[i]*tanTheta, zPlus_vector[i]], q1, r, "inter1")
plotStraightLine ([xMinus_vector[i], xMinus_vector[i]*tanTheta, zMinus_vector[i]], q1, r, "inter2")

i = 1
plotStraightLine ([xPlus_vector[i], xPlus_vector[i]*tanTheta, zPlus_vector[i]], q2, r, "inter3")
plotStraightLine ([xMinus_vector[i], xMinus_vector[i]*tanTheta, zMinus_vector[i]], q2, r, "inter4")


## 2D Plot tools ##

from parabola_plot_tools import parabPlotDoubleProjCones, parabPlotOriginCones
import matplotlib.pyplot as plt
import math
theta = math.atan2((q2 [1] - q1 [1]) , (q2 [0] - q1 [0]))
index = cl.robot.getConfigSize () - 4
NconeOne = [q1 [index]*math.cos(theta) + q1 [index+1]*math.sin(theta), q1 [index+2]]
pointsConeOne = [q1 [0]*math.cos(theta) + q1 [1]*math.sin(theta), q1 [2], xPlus_vector[0]*math.cos(theta) + xPlus_vector[0]*tanTheta*math.sin(theta), zPlus_vector[0], xMinus_vector[0]*math.cos(theta) + xMinus_vector[0]*tanTheta*math.sin(theta), zMinus_vector[0]]

NconeTwo = [q2 [index]*math.cos(theta) + q2 [index+1]*math.sin(theta), q2 [index+2]]
pointsConeTwo = [q2 [0]*math.cos(theta) + q2 [1]*math.sin(theta), q2 [2], xPlus_vector[1]*math.cos(theta) + xPlus_vector[1]*tanTheta*math.sin(theta), zPlus_vector[1], xMinus_vector[1]*math.cos(theta) + xMinus_vector[1]*tanTheta*math.sin(theta), zMinus_vector[1]]

parabPlotDoubleProjCones (cl, 0, theta, NconeOne, pointsConeOne, NconeTwo, pointsConeTwo, plt)

plt.show()

