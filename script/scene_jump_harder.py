#/usr/bin/env python
# Script which goes with animals_description package.
# Easy way to test parabola-planning algo (no internal DoF) on SO3 joint.

from hpp.corbaserver.sphere import Robot
#from hpp.corbaserver.ant import Robot
#from hpp.corbaserver.ant_sphere import Robot
from hpp.corbaserver import Client
from hpp.corbaserver import ProblemSolver
from viewer_display_library import normalizeDir, plotCone, plotFrame, plotThetaPlane, shootNormPlot, plotStraightLine, plotConeWaypoints, plotSampleSubPath, contactPosition, addLight, plotSphere, plotSpheresWaypoints
from parseLog import parseConfig, parseNodes, parseIntersectionConePlane, parseAlphaAngles
from parabola_plot_tools import parabPlotDoubleProjCones, parabPlotOriginCones
import math
import numpy as np

robot = Robot ('robot')
robot.setJointBounds('base_joint_xyz', [-6, 6.8, -2.5, 3.2, 0, 8]) # ultimate goal!
#robot.setJointBounds('base_joint_xyz', [1.6, 6.8, -2.2, 1.5, 0, 3]) # first goal
#robot.setJointBounds('base_joint_xyz', [-0.3, 6.8, -2.2, 2.4, 0, 3]) # second goal
#robot.setJointBounds('base_joint_xyz', [-2.6, 6.8, -2.2, 2.4, 0, 3]) # third goal
#robot.setJointBounds('base_joint_xyz', [-6, 6.8, -2.8, 3.2, 0, 3]) # start to bottom
#robot.setJointBounds('base_joint_xyz', [-6, -2.2, -2.4, 3, 0, 8]) # bottom to ultimate
#robot.setJointBounds('base_joint_xyz', [-5, -2.2, -0.1, 2.8, 0, 6]) # bottom to middle column
#robot.setJointBounds('base_joint_xyz', [-5, -2.2, -0.1, 2.8, 0, 3]) # bottom to bottom 1
#robot.setJointBounds('base_joint_xyz', [-6, 6.8, -2.5, 3.2, 0, 3]) # first to bottom

ps = ProblemSolver (robot)
cl = robot.client
#cl.obstacle.loadObstacleModel('animals_description','inclined_plane_3d','inclined_plane_3d')

from hpp.gepetto import Viewer, PathPlayer
r = Viewer (ps)
pp = PathPlayer (robot.client, r)
r.loadObstacleModel ("animals_description","scene_jump_harder","scene_jump_harder")
addLight (r, [-5,0,2,1,0,0,0], "li"); addLight (r, [2,-2,5,1,0,0,0], "li1")

# Configs : [x, y, z, q1, q2, q3, q4, dir.x, dir.y, dir.z, theta]
q11 = [6.2, 0.5, 0.5, 0, 0, 0, 1, 0, 0, 1, 0] # start
#q11 = [-3.5, 1.7, 0.4, 0, 0, 0, 1, 0, 0, 1, 0] # bottom of column
r(q11)
#q22 = [2.6, -1.4, 0.35, 0, 0, 0, 1, 0, 0, 1, 0] # first goal
#q22 = [0.7, 1.55, 0.4, 0, 0, 0, 1, 0, 0, 1, 0] # second goal
#q22 = [-1.7, -1.5, 0.4, 0, 0, 0, 1, 0, 0, 1, 0] # third goal
#q22 = [-3.5, 1.7, 0.4, 0, 0, 0, 1, 0, 0, 1, 0] # bottom of column
#q22 = [-3.3, 1.5, 3.4, 0, 0, 0, 1, 0, 0, 1, 0] # in column
#q22 = [-4.2, 0.9, 1.7, 0, 0, 0, 1, 0, 0, 1, 0] # bottom 1 of column
#q22 = [-4.4, 0.9, 4.1, 0, 0, 0, 1, 0, 0, 1, 0] # bottom 3 of column
q22 = [-4.4, -1.5, 6.5, 0, 0, 0, 1, 0, 0, 1, 0] # ultimate goal!
r(q22)

q1 = cl.robot.projectOnObstacle (q11, 0.001); q2 = cl.robot.projectOnObstacle (q22, 0.001)

#cl.problem.setFrictionCoef(0.5)
cl.problem.setMaxVelocityLim(7.0)
ps.setInitialConfig (q1); ps.addGoalConfig (q2)
ps.solve ()

samples = plotSampleSubPath (cl, r, 0, 20, "curvy", [0,0,1,1])

samples2 = plotSampleSubPath (cl, r, 2, 20, "curvy2", [0,0.4,0.7,1])

#ps.saveRoadmap ('/local/mcampana/devel/hpp/data/PARAB_envir3d_with_window.rdm')

# start to bottom
ps.resetGoalConfigs ()
robot.setJointBounds('base_joint_xyz', [-6, 6.7, -2.5, 3.2, 0, 3]) # start to bottom
q11 = [5.7, 0.5, 0.5, 0, 0, 0, 1, 0, 0, 1, 0]
q22 = [-3.5, 1.7, 0.4, 0, 0, 0, 1, 0, 0, 1, 0]

cl.problem.generateValidConfig(2)

r.client.gui.setVisibility('robot/l_bounding_sphere',"OFF")



-5.36273,1.76984,3.28844,0.707032,0.0102817,-0.707032,0.0102817,-0.999577,-0.0290779,-0,-4.57484e+238


r(ps.configAtParam(0,0.001))
ps.pathLength(0)
wp = ps.getWaypoints (0)

# Get projected random configs CONES and display them
num_log = 7308
qrands = parseConfig(num_log,'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/parabola/parabola-planner.cc:157: q_proj: ')
sphereNamePrefix="qrand_sphere_"
for i in range(0,len(qrands)):
    qrand = qrands[i]
    #sphereName = sphereNamePrefix+str(i)
    #r.client.gui.addSphere (sphereName,0.1,[0.8,0.1,0.1,1]) # red
    #r.client.gui.applyConfiguration (sphereName, [qrand[0],qrand[1],qrand[2],1,0,0,0])
    #r.client.gui.addToGroup (sphereName, r.sceneName)
    coneName = sphereNamePrefix+str(i)
    qCone = cl.robot.setOrientation (qrand.tolist())
    r.loadObstacleModel ("animals_description","friction_cone2",coneName)
    r.client.gui.applyConfiguration (coneName, qCone[0:7])
    r.client.gui.addToGroup (coneName, r.sceneName)

r.client.gui.refresh ()


# Plot a cone and rotate it!
qCone = cl.robot.setOrientation (q2) # wp[15]
coneName = "cone5"
r.loadObstacleModel ("animals_description","friction_cone2",coneName)
r.client.gui.applyConfiguration (coneName, qCone[0:7])
r.client.gui.refresh ()
#r.client.gui.removeFromGroup (coneName, r.sceneName)

## 3D Plot tools ##
q0 = [0, 0, 5, 0, 0, 0, 1, 0, 0, 1, 0];
r(q0)

plotFrame (r, "frame", [0,0,0], 0.5)

plotCone (q, cl, r, "yep", "friction_cone2")
plotConeWaypoints (cl, 0, r, "wp", "friction_cone2")

plotThetaPlane (q1, q2, r, "ThetaPlane")
r.client.gui.removeFromGroup ("ThetaPlane", r.sceneName)
r.client.gui.removeFromGroup ("ThetaPlanebis", r.sceneName)

#plotCone (q1, cl, r, 0.5, 0.4, "c1"); plotCone (q2, cl, r, 0.5, 0.4, "c2")

index = cl.robot.getConfigSize () - cl.robot.getExtraConfigSize ()
q = q2[::]
plotStraightLine ([q [index],q [index+1],q [index+2]], q, r, "normale2")


## Plot all cone waypoints:
#plotConeWaypoints (cl, 0, r, 0.5, 0.4, "wpcones")
wp = cl.problem.getWaypoints (0)
for i in np.arange(0, len(wp), 1):
    qCone = cl.robot.setOrientation (wp[i])
    coneName = "wp_cone_"+str(i)
    r.loadObstacleModel ("animals_description","friction_cone2",coneName)
    r.client.gui.applyConfiguration (coneName, qCone[0:7])
    r.client.gui.refresh ()


## Plot ONE cone-planeTheta intersection:
num_log = 3533
configs, theta_vector, xPlus_vector, xMinus_vector, zPlus_vector, zMinus_vector = parseIntersectionConePlane (num_log,'220: theta: ', '544: q: ', '545: x_plus: ', '546: x_minus: ', '547: z_x_plus: ', '548: z_x_minus: ', 11)
i = 0
plotStraightLine ([xPlus_vector[i], xPlus_vector[i]*math.tan(theta_vector[i]), zPlus_vector[i]], q11, r, "inter1")
plotStraightLine ([xMinus_vector[i], xMinus_vector[i]*math.tan(theta_vector[i]), zMinus_vector[i]], q11, r, "inter2")

i = 1
plotStraightLine ([xPlus_vector[i], xPlus_vector[i]*math.tan(theta_vector[i]), zPlus_vector[i]], q22, r, "inter33")
plotStraightLine ([xMinus_vector[i], xMinus_vector[i]*math.tan(theta_vector[i]), zMinus_vector[i]], q22, r, "inter44")
#r.client.gui.removeFromGroup ("inter2"+"straight", r.sceneName)


## Plot all cone-planeTheta intersections:
num_log = 7308
configs, theta_vector, xPlus_vector, xMinus_vector, zPlus_vector, zMinus_vector = parseIntersectionConePlane (num_log,'222: theta: ', '549: q: ', '550: x_plus: ', '551: x_minus: ', '552: z_x_plus: ', '553: z_x_minus: ', 11)
len(xPlus_vector)
for i in range(0,len(xPlus_vector)/4):
    plotStraightLine ([xPlus_vector[i], xPlus_vector[i]*math.tan(theta_vector[i]), zPlus_vector[i]], configs[i], r, "inter"+str(i))




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


# get all theta:
for i in range(0,len(wp)):
    print wp[i][10]

# Plot a sphere (dot)
sphereNamePrefix="sdg2"
sphereName = sphereNamePrefix
r.client.gui.addSphere (sphereName,0.01,[0,0.5,0.5,1]) # grey
r.client.gui.applyConfiguration (sphereName, [-4.1308336459234205, 0.6935200067554956, 3.943446921479487,1,0,0,0])
r.client.gui.addToGroup (sphereName, r.sceneName)
r.client.gui.refresh ()

#q11 = [-3.49712,1.7903,0.269499,0.95451,0.297974, -0.0105248,0.00328557,-0.018134,-0.568908,0.822201,0]
#q22 = [-2.94613,-0.761086,4.40897,-0.3043,0.165307, 0.821124,-0.453685,-0.295478,-0.399652,-0.867739,1.17288]

