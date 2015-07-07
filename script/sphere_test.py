#/usr/bin/env python
# Script which goes with animals_description package.
# Easy way to test parabola-planning algo (no internal DoF) on SO3 joint.

from hpp.corbaserver.sphere import Robot
from hpp.corbaserver import Client
from hpp.corbaserver import ProblemSolver
from viewer_display_library import normalizeDir, plotVerticalCone, plotCone, plotPath, plotVerticalConeWaypoints, plotFrame

robot = Robot ('sphere')
#robot.setJointBounds('base_joint_xyz', [-6, 6, -6, 6, -1, 10])
robot.setJointBounds('base_joint_xyz', [-3.9, 3.9, -3.9, 3.9, 1, 12]) # test
ps = ProblemSolver (robot)
cl = robot.client
#cl.obstacle.loadObstacleModel('animals_description','inclined_plane_3d','inclined_plane_3d')

# Configs : [x, y, z, rz, ry, rx, dir.x, dir.y, dir.z] #q2 = [1.5, 5.5, 3.41, 0, 0, 0, 0, 0, 1]
#q1 = [-1.5, -1.5, 3.41, 0, 0, 0, 0, 0, 1]; q2 = [2.6, 3.7, 3.41, 0, 0, 0, 0, 0, 1]
q11 = [-2.8, 2.6, 4.5, 0, 0, 0, 0, 0, 1]; q22 = [2.6, 3, 8, 0, 0, 0, 0, 0, 1]
r(q22)

from hpp.gepetto import Viewer, PathPlayer
r = Viewer (ps)
pp = PathPlayer (robot.client, r)
#r.loadObstacleModel ("animals_description","plane_3d","plane_3d")
r.loadObstacleModel ("animals_description","multiple_planes_3d","multiple_planes_3d")
#r.loadObstacleModel ("animals_description","inclined_plane_3d","inclined_plane_3d")
#r.loadObstacleModel ("animals_description","environment_3d","environment_3d")
r(q11)

# project configs on obst
q1 = cl.robot.projectOnObstacle (q11, 3); q2 = cl.robot.projectOnObstacle (q22, 3)

ps.setInitialConfig (q1); ps.addGoalConfig (q2); ps.solve ()


r( ps.configAtParam(0,0.001) )
ps.pathLength(0)
ps.getWaypoints (0)


## Plot tools ##
plotPath (cl, 0, r, "z", 0.05)

plotCone (q1, cl, r, 0.5, 0.5, "c1")
plotCone (q2, cl, r, 0.5, 0.5, "c2")


plotFrame (r, "_", [-1,0,3.7], 0.5)


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

