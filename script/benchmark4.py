#/usr/bin/env python
# Benchmark4. Script which goes with animals_description package.
# The script launches a point-robot and the scene_jump_harder (mesh version 10) environment.
# It defines init and final configs, and solve them for 2 couples of mu / vmax (parabola constraints).


#from hpp.corbaserver.sphere import Robot
from hpp.corbaserver.sphere_mesh import Robot
from hpp.corbaserver import Client
from hpp.corbaserver import ProblemSolver
from viewer_display_library import normalizeDir, plotCone, plotFrame, plotThetaPlane, shootNormPlot, plotStraightLine, plotConeWaypoints, plotSampleSubPath, contactPosition, addLight, plotSphere, plotSpheresWaypoints
import math
import numpy as np

robot = Robot ('robot')
robot.setJointBounds('base_joint_xyz', [-6, 6.8, -2.5, 3.2, 0, 8])
ps = ProblemSolver (robot)
cl = robot.client

# Configs : [x, y, z, q1, q2, q3, q4, dir.x, dir.y, dir.z, theta]
q11 = [6.2, 0.5, 0.5, 0, 0, 0, 1, 0, 0, 1, 0]; q22 = [-4.4, -1.5, 6.5, 0, 0, 0, 1, 0, 0, 1, 0]
#cl.obstacle.loadObstacleModel('animals_description','scene_jump_harder','')

from hpp.gepetto import Viewer, PathPlayer
r = Viewer (ps); gui = r.client.gui
pp = PathPlayer (robot.client, r)
r.loadObstacleModel ("animals_description","scene_jump_harder","scene_jump_harder")
addLight (r, [-5,0,2,1,0,0,0], "li"); addLight (r, [2,-2,5,1,0,0,0], "li1")
#addLight (r, [-4.4,-2,6.5,1,0,0,0], "li2")
r(q11)

q1 = cl.robot.projectOnObstacle (q11, 0.001); q2 = cl.robot.projectOnObstacle (q22, 0.001)
robot.isConfigValid(q1); robot.isConfigValid(q2)
#r(q1)

ps.setInitialConfig (q1); ps.addGoalConfig (q2)

plotSphere (q2, cl, r, "sphere_q2", [0,1,0,1], 0.02) # same as robot
nPointsPlot = 50
offsetOrientedPath = 2 # If remove oriented path computation in ProblemSolver, set to 1 instead of 2

plotFrame (r, 'frame_group', [0,0,0], 0.6)

# First parabolas: vmax = 7m/s,  mu = 1.2
cl.problem.setFrictionCoef(1.2); cl.problem.setMaxVelocityLim(7)
plotCone (q1, cl, r, "cone_11", "friction_cone2"); plotCone (q2, cl, r, "cone_21", "friction_cone2")
ps.clearRoadmap();
solveTime = ps.solve ()
pahtId = ps.numberPaths()-offsetOrientedPath # path without orientation stuff
samples = plotSampleSubPath (cl, r, pahtId, nPointsPlot, "path0", [0,0,1,1])
plotConeWaypoints (cl, pahtId, r, "cone_wp_group", "friction_cone2")
plotSpheresWaypoints (cl, pahtId, r, "sphere_wp_group", [0,0,1,1], 0.02)
print "solve duration: " + str(solveTime)
print "path length: " + str(ps.pathLength(pahtId))
print "number of waypoints: " + str(len(ps.getWaypoints (pahtId)))
print "number of nodes: " + str(ps.numberNodes ())
cl.problem.getResultValues ()

# Second parabolas: vmax = 6.5m/s,  mu = 0.5
plotCone (q1, cl, r, "cone12", "friction_cone"); plotCone (q2, cl, r, "cone22", "friction_cone")
cl.problem.setFrictionCoef(0.5); cl.problem.setMaxVelocityLim(6.5)
ps.clearRoadmap();
solveTime = ps.solve ()
pahtId = ps.numberPaths()-offsetOrientedPath
samples = plotSampleSubPath (cl, r, pahtId, nPointsPlot, "path2", [0.9,0.1,0.1,1]) # Red
plotConeWaypoints (cl, pahtId, r, "cone_wp_group2", "friction_cone")
plotSpheresWaypoints (cl, pahtId, r, "sphere_wp_group2", [0.9,0.1,0.1,1], 0.02)
print "solve duration: " + str(solveTime)
print "path length: " + str(ps.pathLength(pahtId))
print "number of waypoints: " + str(len(ps.getWaypoints (pahtId)))
print "number of nodes: " + str(ps.numberNodes ())
cl.problem.getResultValues ()

#r.client.gui.removeFromGroup("path0",r.sceneName)
# toSeconds = np.array ([60*60,60,1,1e-3]); np.dot(solveTime,toSeconds)
