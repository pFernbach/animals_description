#/usr/bin/env python
# Script which goes with animals_description or room_description package.
# Easy way to test planning algo (no internal DoF) on SO3 joint.

#from hpp.corbaserver.ant import Robot
#from hpp.corbaserver.ant_sphere import Robot
from hpp.corbaserver.sphere import Robot
from hpp.corbaserver import Client
from hpp.corbaserver import ProblemSolver
import numpy as np
from viewer_display_library import normalizeDir, plotCone, plotFrame, plotThetaPlane, shootNormPlot, plotStraightLine, plotConeWaypoints, plotSampleSubPath
from parseLog import parseNodes, parseIntersectionConePlane, parseAlphaAngles

robot = Robot ('robot')
robot.setJointBounds('base_joint_xyz', [-2.6, 2.6, -6, 4.5, -2.5, 1.6])
ps = ProblemSolver (robot)
cl = robot.client

from hpp.gepetto import Viewer, PathPlayer
r = Viewer (ps)
pp = PathPlayer (robot.client, r)
r.loadObstacleModel ("animals_description","cave","cave")
#cl.obstacle.loadObstacleModel('animals_description','cave','cave')

# Configs : [x, y, z, qw, qx, qy, qz, nx, ny, nz, theta]
q11 = [0.4, -5.2, -0.7, 1, 0, 0, 0, 0, 0, 1, 0] # cave entry
r(q11)
q22 = [-0.18, 3.5, -0.11, 1, 0, 0, 0, 0, 0, 1, 0] # cave end
r(q22)

q1 = cl.robot.projectOnObstacle (q11, 0.005)
robot.isConfigValid(q1)
r(q1)
q2 = cl.robot.projectOnObstacle (q22, 0.005)
robot.isConfigValid(q2)
r(q2)

ps.setInitialConfig (q1); ps.addGoalConfig (q2)
ps.solve ()

samples = plotSampleSubPath (cl, r, 0, 20, "curvy", [0,0.8,0.2,1])

samples = plotSampleSubPath (cl, r, 1, 20, "curvy2", [0,0.85,0.25,1])


#ps.saveRoadmap ('/local/mcampana/devel/hpp/data/PARAB_ant_in_cave1.rdm')


r(ps.configAtParam(0,0.001))
ps.pathLength(0)
wp = ps.getWaypoints (0) 
r.client.gui.setVisibility('robot/l_bounding_sphere',"OFF")

## 3D Plot tools ##
q0 = [0, 0, 5, 0, 0, 0, 1, 0, 0, 1, 0];
r(q0)

plotFrame (r, "frame", [0,0,4], 0.5)

plotThetaPlane (q1, q2, r, "ThetaPlane")

plotCone (q, cl, r, "yep", "friction_cone2")
plotConeWaypoints (cl, 0, r, "wp", "friction_cone")
plotConeWaypoints (cl, 1, r, "wp2", "friction_cone")

index = cl.robot.getConfigSize () - cl.robot.getExtraConfigSize ()
q = q2[::]
plotStraightLine ([q[index], q[index+1], q[index+2]], q, r, "normale")



# --------------------------------------------------------------------#

## Add light to scene ##
lightName = "li5"
r.client.gui.addLight (lightName, r.windowId, 0.001, [0.4,0.4,0.4,1])
r.client.gui.addToGroup (lightName, r.sceneName)
#r.client.gui.applyConfiguration (lightName, [1,0,0,1,0,0,0])
r.client.gui.applyConfiguration (lightName, [0.2,-13.5,-2.3,1,0,0,0])
r.client.gui.applyConfiguration (lightName, [0,-5,-2.5,1,0,0,0])
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
r.client.gui.removeFromGroup("li",r.sceneName)
r.client.gui.getNodeList()
split -l 10000 scene.obj

qt1 = [0.11844751003683414, 3.187439275561228, 0.5110408998245528, 0.05105927086189454, -0.03984712928298416, 0.9793629247909962, -0.1914508257798393, 0.11526864531036812, -0.3709305506445321, -0.9214790643345386]

plotStraightLine ([0.11526864531036812, -0.3709305506445321, -0.9214790643345386], [0.11844751003683414, 3.187439275561228, 0.5110408998245528, 0.05105927086189454, -0.03984712928298416, 0.9793629247909962, -0.1914508257798393, 0.11526864531036812, -0.3709305506445321, -0.9214790643345386], r, "normale1")

plotCone ([0.11844751003683414, 3.187439275561228, 0.5110408998245528, 0.05105927086189454, -0.03984712928298416, 0.9793629247909962, -0.1914508257798393, 0.11526864531036812, -0.3709305506445321, -0.9214790643345386], cl, r, 0.5, 0.4, "c1")

qt2 = [0.21488429128778605, 3.2997394431239395, -0.16451500741630581, 0.8504991679459113, -0.07690347149493239, -0.2167831950901335, -0.47301381345776133, -0.295995045457713, 0.335895568643468, 0.8941818047970825]

plotStraightLine ([-0.295995045457713, 0.335895568643468, 0.8941818047970825], [0.21488429128778605, 3.2997394431239395, -0.16451500741630581, 0.8504991679459113, -0.07690347149493239, -0.2167831950901335, -0.47301381345776133, -0.295995045457713, 0.335895568643468, 0.8941818047970825], r, "normale2")

plotCone ([0.21488429128778605, 3.2997394431239395, -0.16451500741630581, 0.8504991679459113, -0.07690347149493239, -0.2167831950901335, -0.47301381345776133, -0.295995045457713, 0.335895568643468, 0.8941818047970825], cl, r, 0.5, 0.4, "c2")



ps.setInitialConfig (qt1); ps.addGoalConfig (qt2); ps.solve ()

 texcoord="UVMap"
<instance_material symbol="stone14-material" target="#stone14-material">
  <bind_vertex_input semantic="UVMap" input_semantic="TEXCOORD" input_set="0"/>
</instance_material>


tanTheta = (qt2 [1] - qt1 [1]) / (qt2 [0] - qt1 [0])
num_log = 7795
line = 517 # "q: ..." line
configs, xPlus_vector, xMinus_vector, zPlus_vector, zMinus_vector = parseIntersectionConePlane (num_log, str(line)+': q: ', str(line+1)+': x_plus: ', str(line+2)+': x_minus: ', str(line+3)+': z_x_plus: ', str(line+4)+': z_x_minus: ')
i = 0
plotStraightLine ([xPlus_vector[i], xPlus_vector[i]*tanTheta, zPlus_vector[i]], qt1, r, "inter1")
plotStraightLine ([xMinus_vector[i], xMinus_vector[i]*tanTheta, zMinus_vector[i]], qt1, r, "inter2")

i = 1
plotStraightLine ([xPlus_vector[i], xPlus_vector[i]*tanTheta, zPlus_vector[i]], qt2, r, "inter3")
plotStraightLine ([xMinus_vector[i], xMinus_vector[i]*tanTheta, zMinus_vector[i]], qt2, r, "inter4")

# chemin arrivée foireuse: respecte pas 3ème contrainte:

qt1 = [-0.015348906975285266, -4.171305044110657, -0.9392091187458651, 0.9315152701322247, 0.1552333289221375, 0.03148397146866394, -0.32739986964383516, -0.0429911429320373, -0.30982012895828037, 0.9498227462646237]

plotStraightLine ([-0.0429911429320373, -0.30982012895828037, 0.9498227462646237], qt1, r, "normale33")

plotCone (qt1, cl, r, 0.5, 0.3, "c33")

qt2 = [-0.43304741448409084, -4.349158016911722, 0.9838203122978445, -0.11403063817687004, 0.59745220847309, 0.7873253003678657, -0.10083027099166687, -0.30004094907731027, -0.022516733564641164, -0.9536605400174454]

plotStraightLine ([-0.30004094907731027, -0.022516733564641164, -0.9536605400174454], qt2, r, "normale3")

plotCone (qt2, cl, r, 0.5, 0.3, "c3")

qt1 = [-0.015348906975285266, -4.171305044110657, -0.9392091187458651, 0.9315152701322247, 0.1552333289221375, 0.03148397146866394, -0.32739986964383516, -0.0429911429320373, -0.30982012895828037, 0.9498227462646237]
qt2 = [-0.43304741448409084, -4.349158016911722, 0.9838203122978445, -0.11403063817687004, 0.59745220847309, 0.7873253003678657, -0.10083027099166687, -0.30004094907731027, -0.022516733564641164, -0.9536605400174454]
ps.setInitialConfig (qt1); ps.addGoalConfig (qt2); ps.solve ()

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

