#/usr/bin/env python
# Benchmark3. Script which goes with animals_description package.
# The script launches a point-robot and the cave environment.
# It defines init and final configs, and solve them for .. couples of mu / vmax (parabola constraints).


from hpp.corbaserver.sphere import Robot
from hpp.corbaserver import Client
from hpp.corbaserver import ProblemSolver
from viewer_display_library import normalizeDir, plotCone, plotFrame, plotThetaPlane, shootNormPlot, plotStraightLine, plotConeWaypoints, plotSampleSubPath, contactPosition, addLight, plotSphere, plotSpheresWaypoints
from parseLog import parseNodes, parseIntersectionConePlane, parseAlphaAngles
from parabola_plot_tools import parabPlotDoubleProjCones, parabPlotOriginCones
import math
import numpy as np

robot = Robot ('robot')
robot.setJointBounds('base_joint_xyz', [-2.6, 2.6, -6, 4.5, -2.5, 7]) # top end
ps = ProblemSolver (robot)
cl = robot.client

# Configs : [x, y, z, q1, q2, q3, q4, dir.x, dir.y, dir.z, theta]
q11 = [-0.18, 3.5, -0.11, 1, 0, 0, 0, 0, 0, 1, 0];
q22 = [0.27, -5.75, 5.90, 1, 0, 0, 0, 0, 0, 1, 0] # cave top end

from hpp.gepetto import Viewer, PathPlayer
r = Viewer (ps)
pp = PathPlayer (robot.client, r)
r.loadObstacleModel ("animals_description","cave","cave")
addLight (r, [0.2,-13.5,-2.3,1,0,0,0], "li"); addLight (r, [0,-5,-2.5,1,0,0,0], "li1")
r(q11)

q1 = cl.robot.projectOnObstacle (q11, 0.001); q2 = cl.robot.projectOnObstacle (q22, 0.001)
robot.isConfigValid(q1); robot.isConfigValid(q2)
r(q1)

ps.setInitialConfig (q1); ps.addGoalConfig (q2)

plotSphere (q2, cl, r, "sphere_q2", [0,1,0,1], 0.02) # same as robot
nPointsPlot = 50
offsetOrientedPath = 2 # If remove oriented path computation in ProblemSolver, set to 1 instead of 2

# First parabola(s): vmax = 8m/s,  mu = 1.2
plotCone (q1, cl, r, "cone2", "friction_cone2"); plotCone (q2, cl, r, "cone22", "friction_cone2")
solveTime = ps.solve ()
pahtId = ps.numberPaths()-offsetOrientedPath # path without orientation stuff
samples = plotSampleSubPath (cl, r, pahtId, nPointsPlot, "path0", [0,0,1,1])
plotConeWaypoints (cl, pahtId, r, "wp0", "friction_cone2")
plotSpheresWaypoints (cl, pahtId, r, "sphere_wp0", [0,0,1,1], 0.02)
print "solve duration: " + str(solveTime)
print "path length: " + str(ps.pathLength(pahtId))
print "number of waypoints: " + str(len(ps.getWaypoints (pahtId)))
print "number of nodes: " + str(ps.numberNodes ())

# Second parabola(s): vmax = 8m/s,  mu = 0.5
plotCone (q1, cl, r, "cone1", "friction_cone"); plotCone (q2, cl, r, "cone12", "friction_cone")
cl.problem.setFrictionCoef(0.5)
ps.clearRoadmap();
solveTime = ps.solve ()
pahtId = ps.numberPaths()-offsetOrientedPath
samples = plotSampleSubPath (cl, r, pahtId, nPointsPlot, "path1", [0.1,0.8,0.8,1])
plotConeWaypoints (cl, pahtId, r, "wp1", "friction_cone")
plotSpheresWaypoints (cl, pahtId, r, "sphere_wp1", [0.1,0.8,0.8,1], 0.02)
print "solve duration: " + str(solveTime)
print "path length: " + str(ps.pathLength(pahtId))
print "number of waypoints: " + str(len(ps.getWaypoints (pahtId)))
print "number of nodes: " + str(ps.numberNodes ())

# Third parabola(s): vmax = 6.5m/s,  mu = 0.5
cl.problem.setMaxVelocityLim(6.5)
ps.clearRoadmap();
solveTime = ps.solve ()
pahtId = ps.numberPaths()-offsetOrientedPath
samples = plotSampleSubPath (cl, r, pahtId, nPointsPlot, "path2", [0.2,0.8,0.4,1])
plotConeWaypoints (cl, pahtId, r, "wp2", "friction_cone")
plotSpheresWaypoints (cl, pahtId, r, "sphere_wp2", [0.2,0.8,0.4,1], 0.02)
print "solve duration: " + str(solveTime)
print "path length: " + str(ps.pathLength(pahtId))
print "number of waypoints: " + str(len(ps.getWaypoints (pahtId)))
print "number of nodes: " + str(ps.numberNodes ())


## 3D Plot tools ##

#plotFrame (r, "_", [0,0,3.1], 0.5)

#plotThetaPlane (q1, q2, r, "ThetaPlane2")



