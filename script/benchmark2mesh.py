#/usr/bin/env python
# Benchmark2mesh. Script which goes with animals_description package.
# The script launches a point-robot and the environment containing an environment 3d mesh (with windows).
# It defines init and final configs, and solve them for 2 couples of mu / vmax (parabola constraints).

#blender/urdf_to_blender.py -p /local/mcampana/devel/hpp/videos/ -i robot.urdf -o robot_blend.py

from hpp.corbaserver.sphere import Robot
from hpp.corbaserver import Client
from hpp.corbaserver import ProblemSolver
from viewer_display_library import normalizeDir, plotCone, plotFrame, plotThetaPlane, shootNormPlot, plotStraightLine, plotConeWaypoints, plotSampleSubPath, contactPosition, addLight, plotSphere, plotSpheresWaypoints
import math
import numpy as np

robot = Robot ('sphere')
robot.setJointBounds('base_joint_xyz', [-5, 5, -5, 5, -1, 7])
ps = ProblemSolver (robot)
cl = robot.client

# Configs : [x, y, z, q1, q2, q3, q4, dir.x, dir.y, dir.z, theta]
q11 = [-3.8, 2.4, 1.2, 0, 0, 0, 1, 0, 0, 1, 0]; q22 = [3.5, -3.3, 0.4, 0, 0, 0, 1, 0, 0, 1, 0]
#cl.obstacle.loadObstacleModel('animals_description','envir3d_window_mesh','')

from hpp.gepetto import Viewer, PathPlayer
r = Viewer (ps)
pp = PathPlayer (robot.client, r)
r.loadObstacleModel ("animals_description","envir3d_window_mesh","envir3d_window_mesh")
addLight (r, [-3,3,4,1,0,0,0], "li"); addLight (r, [3,-3,4,1,0,0,0], "li1")
r(q11)

q1 = cl.robot.projectOnObstacle (q11, 0.001); q2 = cl.robot.projectOnObstacle (q22, 0.001)
robot.isConfigValid(q1); robot.isConfigValid(q2)
r(q1)

ps.setInitialConfig (q1); ps.addGoalConfig (q2)

plotSphere (q2, cl, r, "sphere_q2", [0,1,0,1], 0.02) # same as robot
nPointsPlot = 50
offsetOrientedPath = 2 # If remove oriented path computation in ProblemSolver, set to 1 instead of 2
plotFrame (r, "_", [0,0,2.8], 0.5)

# First parabola(s): vmax = 6.8m/s,  mu = 1.2
cl.problem.setFrictionCoef(1.2); cl.problem.setMaxVelocityLim(6.8)
plotCone (q1, cl, r, "cone2", "friction_cone2"); plotCone (q2, cl, r, "cone22", "friction_cone2")
ps.clearRoadmap();
solveTime = ps.solve () # 0.085 s
pahtId = ps.numberPaths()-offsetOrientedPath # path without orientation stuff
samples = plotSampleSubPath (cl, r, pahtId, nPointsPlot, "path0", [0,0,1,1])
plotConeWaypoints (cl, pahtId, r, "wp0", "friction_cone2")
plotSpheresWaypoints (cl, pahtId, r, "sphere_wp0", [0,0,1,1], 0.02)
print "solve duration: " + str(solveTime)
print "path length: " + str(ps.pathLength(pahtId))
print "number of waypoints: " + str(len(ps.getWaypoints (pahtId)))
print "number of nodes: " + str(ps.numberNodes ())
cl.problem.getResultValues ()


# Second parabola(s): vmax = 5.3m/s,  mu = 0.5
plotCone (q1, cl, r, "cone1", "friction_cone"); plotCone (q2, cl, r, "cone12", "friction_cone")
cl.problem.setFrictionCoef(0.5); cl.problem.setMaxVelocityLim(5.3)
ps.clearRoadmap();
solveTime = ps.solve () # 0.738 s
pahtId = ps.numberPaths()-offsetOrientedPath
samples = plotSampleSubPath (cl, r, pahtId, nPointsPlot, "path2", [0.2,0.8,0.2,1])
plotConeWaypoints (cl, pahtId, r, "wp2", "friction_cone")
plotSpheresWaypoints (cl, pahtId, r, "sphere_wp2", [0.2,0.8,0.2,1], 0.02)
print "solve duration: " + str(solveTime)
print "path length: " + str(ps.pathLength(pahtId))
print "number of waypoints: " + str(len(ps.getWaypoints (pahtId)))
print "number of nodes: " + str(ps.numberNodes ())
cl.problem.getResultValues ()


## 3D Plot tools ##

#plotThetaPlane (q1, q2, r, "ThetaPlane2")
#r.client.gui.removeFromGroup("path0",r.sceneName)

# toSeconds = np.array ([60*60,60,1,1e-3]); np.dot(solveTime,toSeconds)
