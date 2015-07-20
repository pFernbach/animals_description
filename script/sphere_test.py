#/usr/bin/env python
# Script which goes with animals_description package.
# Easy way to test parabola-planning algo (no internal DoF) on SO3 joint.

from hpp.corbaserver.sphere import Robot
from hpp.corbaserver import Client
from hpp.corbaserver import ProblemSolver
from viewer_display_library import normalizeDir, plotVerticalCone, plotCone, plotPath, plotVerticalConeWaypoints, plotFrame, plotThetaPlane, shootNormPlot, plotStraightLine, plotConeWaypoints
from parseLog import parseNodes, parseIntersectionConePlane, parseAlphaAngles
from parabola_plot_tools import parabPlotDoubleProjCones, parabPlotOriginCones
import math
import numpy as np

robot = Robot ('sphere')
robot.setJointBounds('base_joint_xyz', [-3.9, 3.9, -3.9, 3.9, 1, 12]) # plane
#robot.setJointBounds('base_joint_xyz', [-6, 6, -6, 6, -1, 10]) # environment_3d
ps = ProblemSolver (robot)
cl = robot.client
#cl.obstacle.loadObstacleModel('animals_description','inclined_plane_3d','inclined_plane_3d')

# Configs : [x, y, z, q1, q2, q3, q4, dir.x, dir.y, dir.z]
#q1 = [-1.5, -1.5, 3.41, 0, 0, 0, 1, 0, 0, 1]; q2 = [2.6, 3.7, 3.41, 0, 0, 0, 1, 0, 0, 1]
#q11 = [2.5, 3, 4, 0, 0, 0, 1, 0, 0, 1]; q22 = [-2.5, 3, 4, 0, 0, 0, 1, 0, 0, 1] # plane with theta = Pi
q11 = [-2.5, 3, 4, 0, 0, 0, 1, 0, 0, 1]; q22 = [2.5, 2.7, 4, 0, 0, 0, 1, 0, 0, 1] # plane with theta ~= 0
#q11 = [-5, 3.1, 4.2, 0, 0, 0, 1, 0, 0, 1]; q22 = [5.2, -5.2, 4, 0, 0, 0, 1, 0, 0, 1] # environment_3d
r(q22)

from hpp.gepetto import Viewer, PathPlayer
r = Viewer (ps)
pp = PathPlayer (robot.client, r)
#r.loadObstacleModel ("animals_description","plane_3d","plane_3d")
#r.loadObstacleModel ("animals_description","multiple_planes_3d","multiple_planes_3d")
r.loadObstacleModel ("animals_description","inclined_plane_3d","inclined_plane_3d")
#r.loadObstacleModel ("animals_description","environment_3d","environment_3d")
#r.loadObstacleModel ("animals_description","environment_3d_with_window","environment_3d_with_window")
r(q11)

# project configs on obst
q1 = cl.robot.projectOnObstacle (q11, 3); q2 = cl.robot.projectOnObstacle (q22, 3)

ps.setInitialConfig (q1); ps.addGoalConfig (q2); ps.solve ()


r( ps.configAtParam(0,0.001) )
ps.pathLength(0)
ps.getWaypoints (0)



## 3D Plot tools ##
q0 = [0, 0, 5, 0, 0, 0, 1, 0, 0, 1];
r(q0)

plotPath (cl, 0, r, "path", 0.4)

plotThetaPlane (q1, q2, r, "ThetaPlane2")

plotCone (q1, cl, r, 0.5, 0.4, "c1")
plotCone (q2, cl, r, 0.5, 0.4, "c2")
index = cl.robot.getConfigSize () - 3
plotStraightLine ([q1 [index],q1 [index+1],q1 [index+2]], q1, r, "normale1")
plotStraightLine ([q2 [index],q2 [index+1],q2 [index+2]], q2, r, "normale2")

plotConeWaypoints (cl, 0, r, 0.5, 0.4, "wpcones")
wp = cl.problem.getWaypoints (0)
for i in np.arange(0, len(wp), 1):
    plotCone (wp[i], cl, r, 0.5, 0.4, "wpcones"+str(i))



plotFrame (r, "_", [-1,0,3.7], 0.5)


tanTheta = (q2 [1] - q1 [1]) / (q2 [0] - q1 [0])
num_log = 22393
configs, xPlus_vector, xMinus_vector, zPlus_vector, zMinus_vector = parseIntersectionConePlane (num_log, '479: q: ', '480: x_plus: ', '481: x_minus: ', '482: z_x_plus: ', '483: z_x_minus: ')
i = 0
plotStraightLine ([xPlus_vector[i], xPlus_vector[i]*tanTheta, zPlus_vector[i]], q1, r, "inter1")
plotStraightLine ([xMinus_vector[i], xMinus_vector[i]*tanTheta, zMinus_vector[i]], q1, r, "inter2")

i = 1
plotStraightLine ([xPlus_vector[i], xPlus_vector[i]*tanTheta, zPlus_vector[i]], q2, r, "inter33")
plotStraightLine ([xMinus_vector[i], xMinus_vector[i]*tanTheta, zMinus_vector[i]], q2, r, "inter44")

r.client.gui.removeFromGroup ("inter2"+"straight", r.sceneName)

## 2D Plot tools ##

import matplotlib.pyplot as plt
theta = math.atan2((q2 [1] - q1 [1]) , (q2 [0] - q1 [0]))
index = cl.robot.getConfigSize () - 3
NconeOne = [q1 [index]*math.cos(theta) + q1 [index+1]*math.sin(theta), q1 [index+2]]
pointsConeOne = [q1 [0]*math.cos(theta) + q1 [1]*math.sin(theta), q1 [2], xPlus_vector[0]*math.cos(theta) + xPlus_vector[0]*tanTheta*math.sin(theta), zPlus_vector[0], xMinus_vector[0]*math.cos(theta) + xMinus_vector[0]*tanTheta*math.sin(theta), zMinus_vector[0]]

NconeTwo = [q2 [index]*math.cos(theta) + q2 [index+1]*math.sin(theta), q2 [index+2]]
pointsConeTwo = [q2 [0]*math.cos(theta) + q2 [1]*math.sin(theta), q2 [2], xPlus_vector[1]*math.cos(theta) + xPlus_vector[1]*tanTheta*math.sin(theta), zPlus_vector[1], xMinus_vector[1]*math.cos(theta) + xMinus_vector[1]*tanTheta*math.sin(theta), zMinus_vector[1]]

parabPlotDoubleProjCones (cl, 0, theta, NconeOne, pointsConeOne, NconeTwo, pointsConeTwo, plt)

plt.show()


angles = parseAlphaAngles (num_log, '285: alpha_0_min: ', '286: alpha_0_max: ', '303: alpha_lim_minus: ', '302: alpha_lim_plus: ', '317: alpha_imp_inf: ', '318: alpha_imp_sup: ', '290: alpha_inf4: ')

i = 0
parabPlotOriginCones (cl, 0, theta, NconeOne, pointsConeOne, angles, i, 0.6, plt)
plt.show()


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


