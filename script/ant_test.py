#/usr/bin/env python
# Script which goes with animals_description package.
# Easy way to test planning algo (no internal DoF) on SO3 joint.

from hpp.corbaserver.ant import Robot
from hpp.corbaserver import Client
from hpp.corbaserver import ProblemSolver
import numpy as np
from viewer_display_library import normalizeDir, plotVerticalCone, plotCone, plotPath, plotVerticalConeWaypoints, plotFrame, plotThetaPlane, shootNormPlot, plotStraightLine, plotConeWaypoints

robot = Robot ('robot')
robot.setJointBounds('base_joint_xyz', [-5, 5, -5, 5, -0.01, 3])
ps = ProblemSolver (robot)
cl = robot.client

# Configs : [x, y, z, rz, ry, rx, (gamma)]
#q11 = [0, -1, 0.002, 0, 0, 0, 0, 0, 1]; #q22 = [-0.1, 1.75, 0.302, 1.57, 0, 0, 0, 0, 1]
q11 = [1.781, 1.4, 1.3, 0, 0, 0, 1, 0, 0, 1]; #q22 = [0.8, -2.6, 2.35, 0, 0, 0, 1, 0, 0, 1]
#q22 = [0.45, 1.4, 0.8, 0, 0, 0, 1, 0, 0, 1] # easier
q22 = [0, 2, 0.23, 1, 0, 0, 0, 0, 0, 1] # under table

#cl.obstacle.loadObstacleModel('room_description','room','')

from hpp.gepetto import Viewer, PathPlayer
r = Viewer (ps)
pp = PathPlayer (robot.client, r)
r.loadObstacleModel ("room_description","room","room")
r(q22)

robot.isConfigValid(q11)
cl.problem.generateValidConfig(2)


qt = [2.0495390217629836, -1.8834072530719876, 1.1419356974733574, 0.2812471039601126, 0.09619100050038619, 0.8211573683900153, -0.4871836761177133, -0.919918, 0.0424716, 0.389805]
qtest = [2.04954, -1.88341, 1.14194, 0.906174, 0.136197, 0.0465817, 0.397655, -0.919918, 0.0424716, 0.389805]

index = cl.robot.getConfigSize () - 3


plotStraightLine ([-0.919918,0.0424716,0.389805], [2.0495390217629836, -1.8834072530719876, 1.1419356974733574, 0.2812471039601126, 0.09619100050038619, 0.8211573683900153, -0.4871836761177133, -0.919918, 0.0424716, 0.389805], r, "normale1")

#


q1 = cl.robot.projectOnObstacle (q11, 3); q2 = cl.robot.projectOnObstacle (q22, 3)
r(q2)

ps.setInitialConfig (q1); ps.addGoalConfig (q2); ps.solve ()

# Load box obstacle in HPP for collision avoidance
#cl.obstacle.loadObstacleModel('puzzle_description','decor_very_easy','')


q22 = [0, 2, 0.23, 1, 0, 0, 0, 0, 0, 1]
r(q22)
q2 = cl.robot.projectOnObstacle (q22, 3)
r(q2)

r([1.9055415039823447, 1.4001395451251157, 0.3000000121119993, 0.0, 0.02517383406203692, 0.9996830888205736, 2.1849954283811942e-06, 0.9996830888205736, 0.02517383406203692, 2.1849954283811942e-06])

r([1.9055415039823447, 1.4001395451251157, 0.3000000121119993, 0, 0, 0.707, 0.707, 0.9996830888205736, 0.02517383406203692, 2.1849954283811942e-06])

r( ps.configAtParam(0,2) )
ps.pathLength(0)
ps.getWaypoints (0)

plotPath (cl, 0, r, "pathy", 0.1)

plotFrame (r, "framy", [0,0,1], 0.5)

r.startCapture ("capture","png")
pp(1)
r.stopCapture ()
#ffmpeg -r 50 -i capture_0_%d.png -r 25 -vcodec libx264 video.mp4


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




 :38: q_init: 1.80959,1.23596,1.05975,0,0,0,0.349229,-0.0687689,0.93451,
 :39: q_goal: 0.8,-2.6,2.33838,0,0,0,0,0,1,
 :41: g_: 9.81 , mu_: 0.5 , V0max: 10
 :220: x_0: 1.80959
 :221: y_0: 1.23596
 :222: z_0: 1.05975
 :223: x_imp: 0.8
 :224: y_imp: -2.6
 :225: z_imp: 2.33838
 :226: X: -1.00959
 :227: Y: -3.83596
 :228: Z: 1.27863
 :229: theta: -1.82815
 :230: x_theta_0: -1.65584
 :231: X_theta: 3.96659
 :232: phi: 0.463648
 :453: discr: 1.4242
 :479: q: 1.80959,1.23596,1.05975,0,0,0,0.349229,-0.0687689,0.93451,
 :480: x_plus: -1
 :481: x_minus: 1
 :482: z_x_plus: 14.1441
 :483: z_x_minus: 11.9021
 :493: cos(2*delta): 0.83106
 :246: delta1: 0.294893
 :261: delta2: 0.463648
 :268: gamma_0: 0.0239473
 :269: gamma_imp: 0
 :280: corrected gamma_imp: 0
 :285: alpha_0_min: 1.29985
 :286: alpha_0_max: 1.88964
 :290: alpha_inf4: 0.311833
 :294: alpha_imp_min: -2.03444
 :295: alpha_imp_max: -1.10715
 :302: alpha_lim_plus: 1.35476
 :303: alpha_lim_minus: 0.527865
 :317: alpha_imp_inf: 1.2093
 :318: alpha_imp_sup: -0.93512
 :341: alpha_inf_bound: 1.29985
 :342: alpha_sup_bound: 1.35476
 :352: alpha: 1.32731
 :358: V0: 9.50747
 :368: coefs: -0.93358 0.933765  5.16561 -1.82815  -5.6396
 :570: length = 4.27767

