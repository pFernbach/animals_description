#/usr/bin/env python
# Script which goes with animals_description package.
# Easy way to test parabola-planning algo (no internal DoF) on SO3 joint.

from hpp.corbaserver.sphere import Robot
#from hpp.corbaserver.ant import Robot
#from hpp.corbaserver.ant_sphere import Robot
#from hpp.corbaserver.monopod_mesh import Robot
from hpp.corbaserver import Client
from hpp.corbaserver import ProblemSolver
from viewer_display_library import normalizeDir, plotCone, plotFrame, plotThetaPlane, shootNormPlot, plotStraightLine, plotConeWaypoints, plotSampleSubPath, contactPosition, addLight, plotSphere, plotSpheresWaypoints
from parseLog import parseNodes, parseIntersectionConePlane, parseAlphaAngles
from parabola_plot_tools import parabPlotDoubleProjCones, parabPlotOriginCones
import math
import numpy as np

robot = Robot ('robot')
robot.setJointBounds('base_joint_xyz', [-3.9, 3.9, -3.9, 3.9, 2, 9])
ps = ProblemSolver (robot)
cl = robot.client

# Configs : [x, y, z, q1, q2, q3, q4, dir.x, dir.y, dir.z, theta]
#q1 = [-1.5, -1.5, 3.41, 0, 0, 0, 1, 0, 0, 1, 0]; q2 = [2.6, 3.7, 3.41, 0, 0, 0, 1, 0, 0, 1, 0]
#q11 = [2.5, 3, 4, 0, 0, 0, 1, 0, 0, 1, 0]; q22 = [-2.5, 3, 4, 0, 0, 0, 1, 0, 0, 1, 0] # theta = Pi
q11 = [-2.5, 3, 4, 0, 0, 0, 1, 0, 0, 1, 0]; q22 = [2.5, 2.7, 8, 0, 0, 0, 1, 0, 0, 1, 0] # theta ~= 0
#q11 = [-2.5, 3, 4, 0, 0, 0, 1,-0.1, 0, 0, 1, 0]; q22 = [2.5, 2.7, 8, 0, 0, 0, 1, -0.1, 0, 0, 1, 0] # theta ~= 0 MONOPOD
#r(q22)

from hpp.gepetto import Viewer, PathPlayer
r = Viewer (ps)
pp = PathPlayer (robot.client, r)
r.loadObstacleModel ("animals_description","plane_3d","plane_3d")
addLight (r, [-3,3,7,1,0,0,0], "li"); addLight (r, [3,-3,7,1,0,0,0], "li1")
r(q11)

q1 = cl.robot.projectOnObstacle (q11, 0.001); q2 = cl.robot.projectOnObstacle (q22, 0.001)
robot.isConfigValid(q1); robot.isConfigValid(q2)

ps.setInitialConfig (q1); ps.addGoalConfig (q2)
cl.problem.setMaxVelocityLim(6.5)
cl.problem.setFrictionCoef(0.5)
ps.solve ()

samples = plotSampleSubPath (cl, r, 0, 20, "curvy", [0,0.2,1,1])
samples = plotSampleSubPath (cl, r, ps.numberPaths()-2, 20, "curvy1", [0.2,0.2,0.8,1])
samples = plotSampleSubPath (cl, r, ps.numberPaths()-2, 20, "curvy2", [0.2,0.8,0.2,1])

plotCone (q1, cl, r, "cone1", "friction_cone"); plotCone (q2, cl, r, "cone12", "friction_cone")
plotCone (q1, cl, r, "cone2", "friction_cone2"); plotCone (q2, cl, r, "cone22", "friction_cone2")

plotConeWaypoints (cl, 0, r, "wp", "friction_cone")

r.client.gui.setVisibility('robot/l_bounding_sphere',"OFF")

r(ps.configAtParam(0,0.001))
ps.pathLength(0)
ps.getWaypoints (0)

ps.clearRoadmap()

## 3D Plot tools ##
q0 = [0, 0, 5, 0, 0, 0, 1, 0, 0, 1, 0];
r(q0)

plotFrame (r, "_", [0,0,4], 0.5)

plotPath (cl, 0, r, "pathy", 0.1)

plotThetaPlane (q1, q2, r, "ThetaPlane2")

plotSphere (q2, cl, r, "sphere1", [0,1,0,1], 0.1)

index = cl.robot.getConfigSize () - cl.robot.getExtraConfigSize ()
q = qa[::]
q = [-4.77862,-1.56995,2.87339,-0.416537,-0.469186,-0.619709,0.471511,-0.197677,-0.0998335,0.97517,0.619095]

qprojCorba=[-4.778619492059025, -1.5699444231861588, 2.873387956706481, 0.9470998051218645, 0.017748399125124163, -0.10999926666084152, 0.3009769340010935, -0.19767685053184691, -0.0998334947491579, 0.9751703113251448, 0.619095]
plotStraightLine ([q [index],q [index+1],q [index+2]], q, r, "normale2")

plotConeWaypoints (cl, 0, r, 0.5, 0.4, "wpcones")
wp = cl.problem.getWaypoints (0)
for i in np.arange(0, len(wp), 1):
    plotCone (wp[i], cl, r, 0.5, 0.4, "wpcones"+str(i))


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
import time
pp.dt = 0.01; pp.speed=0.5
r(q1)
r.startCapture ("capture","png")
r(q1); time.sleep(0.2); r(q1)
pp(0)
#pp(ps.numberPaths()-1)
r(q2); time.sleep(1);
r.stopCapture ()

## ffmpeg commands
ffmpeg -r 50 -i capture_0_%d.png -r 25 -vcodec libx264 video.mp4
x=0; for i in *png; do counter=$(printf %04d $x); ln "$i" new"$counter".png; x=$(($x+1)); done
ffmpeg -r 50 -i new%04d.png -r 25 -vcodec libx264 video.mp4
mencoder video.mp4 -channels 6 -ovc xvid -xvidencopts fixed_quant=4 -vf harddup -oac pcm -o video.avi
ffmpeg -i untitled.mp4 -vcodec libx264 -crf 24 video.mp4

# --------------------------------------------------------------------#

## Export to Blender ##
r.client.gui.writeNodeFile(0, 'scene.osg')
# osgconvd -O NoExtras scene.osg scene.dae
from hpp.gepetto.blender.exportmotion import exportStates, exportPath
exportPath(r, cl.robot, cl.problem, 0, 1, 'path2.txt')
exportStates(r, cl.robot, q11, 'configs.txt')
from gepettoimport import loadmotion
loadmotion('/local/mcampana/devel/hpp/videos/path2.txt') # and rename first node manually ...

# --------------------------------------------------------------------#
## DEBUG commands
cl.obstacle.getObstaclePosition('decor_base')
robot.isConfigValid(q1)
robot.setCurrentConfig(q1)
res=robot.distancesToCollision()
r( ps.configAtParam(0,5) )
ps.optimizePath (0)
ps.clearRoadmap ()
ps.resetGoalConfigs ()
from numpy import *
argmin(robot.distancesToCollision()[0])
robot.getJointNames ()
robot.getConfigSize ()
r.client.gui.getNodeList()


['0_scene_hpp_', 'inclined_plane_3d', 'inclined_plane_3d/l_one', 'inclined_plane_3d/l_one_0', 'inclined_plane_3d/obstacle_base', 'inclined_plane_3d/obstacle_base_0', 'robot', 'robot/base_link', 'robot/base_link_0']

# get all theta:
for i in range(0,len(wp)):
    print wp[i][10]

# Plot a sphere (dot)
sphereNamePrefix="sdg"
sphereName = sphereNamePrefix
r.client.gui.addSphere (sphereName,0.01,[0,0.5,0.5,1]) # grey
r.client.gui.applyConfiguration (sphereName, [0.793416,0.98263,7.61459,1,0,0,0])
r.client.gui.addToGroup (sphereName, r.sceneName)
r.client.gui.refresh ()


