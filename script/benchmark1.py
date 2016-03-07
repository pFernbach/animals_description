#/usr/bin/env python
# Benchmark1. Script which goes with animals_description package.
# The script launches a point-robot and the environment containing a simple plane.
# It defines init and final configs, and solve them for three couples of mu / vmax (parabola constraints).

from hpp.corbaserver.sphere import Robot
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
q11 = [-1.8, 3, 4, 0, 0, 0, 1, 0, 0, 1, 0]; q22 = [1.8, 2.7, 8, 0, 0, 0, 1, 0, 0, 1, 0] # theta ~= 0

from hpp.gepetto import Viewer, PathPlayer
r = Viewer (ps)
pp = PathPlayer (robot.client, r)
r.loadObstacleModel ("animals_description","plane_3d","plane_3d")
addLight (r, [-3,3,7,1,0,0,0], "li"); addLight (r, [3,-3,7,1,0,0,0], "li1")
r(q11)

q1 = cl.robot.projectOnObstacle (q11, 0.001); q2 = cl.robot.projectOnObstacle (q22, 0.001)
robot.isConfigValid(q1); robot.isConfigValid(q2)
r(q1)

ps.setInitialConfig (q1); ps.addGoalConfig (q2)

plotSphere (q2, cl, r, "sphere1", [0,1,0,1], 0.02)
nPointsPlot = 60
offsetOrientedPath = 2 # If remove oriented path computation in ProblemSolver, set to 1 instead of 2

plotFrame (r, "_", [0,0,3.1], 0.5)

# First parabola: vmax = 7m/s,  mu = 1.2
cl.problem.setFrictionCoef(1.2); cl.problem.setMaxVelocityLim(7)
plotCone (q1, cl, r, "cone2", "friction_cone2"); plotCone (q2, cl, r, "cone22", "friction_cone2")
solveTime = ps.solve ()
pahtId = ps.numberPaths()-offsetOrientedPath # path without orientation stuff
samples = plotSampleSubPath (cl, r, pahtId, nPointsPlot, "path0", [0,0,1,1])
plotConeWaypoints (cl, pahtId, r, "wp0", "friction_cone2")
plotSpheresWaypoints (cl, pahtId, r, "sphere_wp0", [0,0,1,1], 0.02)
print "solve duration: " + str(solveTime)
print "path length: " + str(ps.pathLength(pahtId))
print "number of waypoints: " + str(len(ps.getWaypoints (pahtId)))

# Second parabola: vmax = 7m/s,  mu = 0.5
plotCone (q1, cl, r, "cone1", "friction_cone"); plotCone (q2, cl, r, "cone12", "friction_cone")
cl.problem.setFrictionCoef(0.5); cl.problem.setMaxVelocityLim(7)
ps.clearRoadmap();
solveTime = ps.solve ()
pahtId = ps.numberPaths()-offsetOrientedPath
samples = plotSampleSubPath (cl, r, pahtId, nPointsPlot, "path1", [0.1,0.8,0.8,1])
plotConeWaypoints (cl, pahtId, r, "wp1", "friction_cone")
plotSpheresWaypoints (cl, pahtId, r, "sphere_wp1", [0.1,0.8,0.8,1], 0.02)
print "solve duration: " + str(solveTime)
print "path length: " + str(ps.pathLength(pahtId))
print "number of waypoints: " + str(len(ps.getWaypoints (pahtId)))

# Third parabola(s): vmax = 6.2m/s,  mu = 0.5
cl.problem.setFrictionCoef(0.5); cl.problem.setMaxVelocityLim(6.0)
ps.clearRoadmap();
solveTime = ps.solve ()
pahtId = ps.numberPaths()-offsetOrientedPath
samples = plotSampleSubPath (cl, r, pahtId, nPointsPlot, "path2", [0.2,0.8,0.2,1])
plotConeWaypoints (cl, pahtId, r, "wp2", "friction_cone")
plotSpheresWaypoints (cl, pahtId, r, "sphere_wp2", [0.2,0.8,0.2,1], 0.02)
print "solve duration: " + str(solveTime)
print "path length: " + str(ps.pathLength(pahtId))
print "number of waypoints: " + str(len(ps.getWaypoints (pahtId)))

cl.problem.getResultValues ()

## 3D Plot tools ##

addLight (r, [0,0,4,1,0,0,0], "li0"); addLight (r, [0,0,4,1,0,0,0], "li00"); addLight (r, [0,0,4,1,0,0,0], "li000")

#plotThetaPlane (q1, q2, r, "ThetaPlane2")


