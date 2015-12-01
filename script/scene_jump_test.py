#/usr/bin/env python
# Script which goes with animals_description package.
# Easy way to test parabola-planning algo (no internal DoF) on SO3 joint.

from hpp.corbaserver.sphere import Robot
#from hpp.corbaserver.ant import Robot
#from hpp.corbaserver.ant_sphere import Robot
from hpp.corbaserver import Client
from hpp.corbaserver import ProblemSolver
from viewer_display_library import normalizeDir, plotVerticalCone, plotCone, plotPath, plotVerticalConeWaypoints, plotFrame, plotThetaPlane, shootNormPlot, plotStraightLine, plotConeWaypoints, plotSampleSubPath
from parseLog import parseNodes, parseIntersectionConePlane, parseAlphaAngles
from parabola_plot_tools import parabPlotDoubleProjCones, parabPlotOriginCones
import math
import numpy as np

robot = Robot ('robot')
#robot.setJointBounds('base_joint_xyz', [-6, 7, -4, 4, 0, 8]) # ultimate goal!
#robot.setJointBounds('base_joint_xyz', [1.2, 7, -3, -0.5, 0, 2.2]) # first goal
robot.setJointBounds('base_joint_xyz', [0, 7, -3, 2.4, 0, 2.2]) # second goal
ps = ProblemSolver (robot)
cl = robot.client
#cl.obstacle.loadObstacleModel('animals_description','inclined_plane_3d','inclined_plane_3d')

from hpp.gepetto import Viewer, PathPlayer
r = Viewer (ps)
pp = PathPlayer (robot.client, r)
r.loadObstacleModel ("animals_description","scene_jump","scene_jump")

# Configs : [x, y, z, q1, q2, q3, q4, dir.x, dir.y, dir.z, theta]
q11 = [6, -1.6, 0.5, 0, 0, 0, 1, 0, 0, 1, 0]
#q22 = [2.2, -2.7, 0.8, 0, 0, 0, 1, 0, 0, 1, 0] # first goal
q22 = [1.2, 1.4, 0.5, 0, 0, 0, 1, 0, 0, 1, 0] # second goal
r(q22)
#q22 = [-4.4, -1.6, 6.5, 0, 0, 0, 1, 0, 0, 1, 0] # ultimate goal!


q1 = cl.robot.projectOnObstacle (q11, 0.02); q2 = cl.robot.projectOnObstacle (q22, 0.02)

ps.setInitialConfig (q1); ps.addGoalConfig (q2); ps.solve ()


#ps.saveRoadmap ('/local/mcampana/devel/hpp/data/PARAB_envir3d_with_window.rdm')

r.client.gui.setVisibility('robot/l_bounding_sphere',"OFF")

samples = plotSampleSubPath (cl, r, 0, 20, "curvy", [0,0,1,1])

r(ps.configAtParam(0,0.001))
ps.pathLength(0)
ps.getWaypoints (0)


qwp1=[6.588526674447825, -1.8345785084341553, 0.36443200826644895, 0.1654550896480227, 0.07312786003135581, 0.25630565685312806, -0.9495179512057083, 5.547415126403182e-16, 0.0, 1.0, -1.5231360759319517]

qwp2=[6.376949224610323, -1.8304419228017523, 0.36443200826644895, -0.10812814084986927, 0.8453266741499781, -0.1992256556515323, 0.4837770739267257, 0.0, -1.486905914403694e-16, 1.0, -2.0916445213810797]

ps.setInitialConfig (qwp1); ps.addGoalConfig (qwp2); ps.solve ()


# Plot a cone and rotate it!
qCone = cl.robot.setOrientation (wp[6])
coneName = "cone3"
r.loadObstacleModel ("animals_description","friction_cone",coneName)
r.client.gui.applyConfiguration (coneName, qCone[0:7])
r.client.gui.refresh ()
#r.client.gui.removeFromGroup (coneName, r.sceneName)

qRobot = q1[::]
coneNamePrefix="cony2"; coneName = coneNamePrefix; height = 0.18
r.client.gui.addCone (coneName,height*0.5,height,[0,0.5,0.5,0.2]) # grey
r.client.gui.addToGroup (coneName, r.sceneName)
r.client.gui.applyConfiguration (coneName, qRobot[0:7])
r.client.gui.refresh ()
#r.client.gui.removeFromGroup (coneName, r.sceneName)

# cone orientation seems not to be good....
r.client.gui.applyConfiguration (coneName, [6, -1.6, 0.5, 0, 1, 0, 0])
r.client.gui.refresh ()

## 3D Plot tools ##
q0 = [0, 0, 5, 0, 0, 0, 1, 0, 0, 1, 0];
r(q0)

plotFrame (r, "frame", [0,0,0], 0.5)

plotPath (cl, 0, r, "pathy", 0.1)

plotThetaPlane (q1, q2, r, "ThetaPlane")
r.client.gui.removeFromGroup ("ThetaPlane", r.sceneName)
r.client.gui.removeFromGroup ("ThetaPlanebis", r.sceneName)

plotCone (q1, cl, r, 0.5, 0.4, "c1")
plotCone (q2, cl, r, 0.5, 0.4, "c2")

index = cl.robot.getConfigSize () - cl.robot.getExtraConfigSize ()
q = qa[::]
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

## Add light to scene ##
lightName = "li"
r.client.gui.addLight (lightName, r.windowId, 0.001, [0.5,0.5,0.5,1])
r.client.gui.addToGroup (lightName, r.sceneName)
r.client.gui.applyConfiguration (lightName, [-2,2,5,1,0,0,0])
#r.client.gui.applyConfiguration (lightName, [2,-2,5,1,0,0,0])
r.client.gui.refresh ()
#r.client.gui.removeFromGroup (lightName, r.sceneName)

# --------------------------------------------------------------------#
## Video capture ##
r.startCapture ("capture","png")
pp(1)
r.stopCapture ()
#ffmpeg -r 50 -i capture_0_%d.png -r 25 -vcodec libx264 video.mp4

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


