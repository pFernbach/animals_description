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
robot.setJointBounds('base_joint_xyz', [-2, 2, -2, 2, -0.1, 8])
ps = ProblemSolver (robot)
cl = robot.client

# Configs : [x, y, z, qw, qx, qy, qz, nx, ny, nz, theta]
q1 = [0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0]
q2 = [1, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0]


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




ps.setInitialConfig (qt1); ps.addGoalConfig (qt2); ps.solve ()

 texcoord="UVMap"
<instance_material symbol="stone14-material" target="#stone14-material">
  <bind_vertex_input semantic="UVMap" input_semantic="TEXCOORD" input_set="0"/>
</instance_material>


tanTheta = (q2 [1] - q1 [1]) / (q2 [0] - q1 [0])
num_log = 7795
line = 517 # "q: ..." line
configs, xPlus_vector, xMinus_vector, zPlus_vector, zMinus_vector = parseIntersectionConePlane (num_log, str(line)+': q: ', str(line+1)+': x_plus: ', str(line+2)+': x_minus: ', str(line+3)+': z_x_plus: ', str(line+4)+': z_x_minus: ')
i = 0
plotStraightLine ([xPlus_vector[i], xPlus_vector[i]*tanTheta, zPlus_vector[i]], qt1, r, "inter1")
plotStraightLine ([xMinus_vector[i], xMinus_vector[i]*tanTheta, zMinus_vector[i]], qt1, r, "inter2")

i = 1
plotStraightLine ([xPlus_vector[i], xPlus_vector[i]*tanTheta, zPlus_vector[i]], qt2, r, "inter3")
plotStraightLine ([xMinus_vector[i], xMinus_vector[i]*tanTheta, zMinus_vector[i]], qt2, r, "inter4")


## 2D Plot tools ##

import matplotlib.pyplot as plt
theta = math.atan2((qt2 [1] - qt1 [1]) , (qt2 [0] - qt1 [0]))
index = cl.robot.getConfigSize () - 3
NconeOne = [qt1 [index]*math.cos(theta) + qt1 [index+1]*math.sin(theta), qt1 [index+2]]
pointsConeOne = [qt1 [0]*math.cos(theta) + qt1 [1]*math.sin(theta), qt1 [2], xPlus_vector[0]*math.cos(theta) + xPlus_vector[0]*tanTheta*math.sin(theta), zPlus_vector[0], xMinus_vector[0]*math.cos(theta) + xMinus_vector[0]*tanTheta*math.sin(theta), zMinus_vector[0]]

NconeTwo = [qt2 [index]*math.cos(theta) + qt2 [index+1]*math.sin(theta), qt2 [index+2]]
pointsConeTwo = [qt2 [0]*math.cos(theta) + qt2 [1]*math.sin(theta), qt2 [2], xPlus_vector[1]*math.cos(theta) + xPlus_vector[1]*tanTheta*math.sin(theta), zPlus_vector[1], xMinus_vector[1]*math.cos(theta) + xMinus_vector[1]*tanTheta*math.sin(theta), zMinus_vector[1]]

parabPlotDoubleProjCones (cl, 0, theta, NconeOne, pointsConeOne, NconeTwo, pointsConeTwo, plt)

plt.show()

