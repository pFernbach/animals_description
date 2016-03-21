#/usr/bin/env python
# Benchmark3. Script which goes with animals_description package.
# The script launches a point-robot and the cave_mono_short environment.
# It defines init and final configs, and solve them for 2 couples of mu / vmax (parabola constraints).

from hpp.corbaserver.sphere import Robot
from hpp.corbaserver import Client
from hpp.corbaserver import ProblemSolver
from viewer_display_library import normalizeDir, plotCone, plotFrame, plotThetaPlane, shootNormPlot, plotStraightLine, plotConeWaypoints, plotSampleSubPath, contactPosition, addLight, plotSphere, plotSpheresWaypoints, plotConesRoadmap
import math
import numpy as np

robot = Robot ('robot')
robot.setJointBounds('base_joint_xyz', [-2.6, 2.6, -3, 4.2, -2.5, 4])
ps = ProblemSolver (robot)
cl = robot.client

# Configs : [x, y, z, q1, q2, q3, q4, dir.x, dir.y, dir.z, theta]
q11 = [-0.3, 3.65, -0.25, 1, 0, 0, 0, 0, 0, 1, 0];
q22 = [-0.18, -2.2, -0.4, 1, 0, 0, 0, 0, 0, 1, 0]
#cl.obstacle.loadObstacleModel('animals_description','cave','')

from hpp.gepetto import Viewer, PathPlayer
r = Viewer (ps); gui = r.client.gui
pp = PathPlayer (robot.client, r)
r.loadObstacleModel ("animals_description","cave","cave")
#addLight (r, [-0.3, 3.8, 0,1,0,0,0], "li"); addLight (r, [-0.18, -3, 0.1,1,0,0,0], "li1"); addLight (r, [-0.3, 4, 0,1,0,0,0], "li3")
r(q11)

q1 = cl.robot.projectOnObstacle (q11, 0.001); q2 = cl.robot.projectOnObstacle (q22, 0.001)
robot.isConfigValid(q1); robot.isConfigValid(q2)
r(q1)

ps.setInitialConfig (q1); ps.addGoalConfig (q2)

offsetOrientedPath = 2 # If remove oriented path computation in ProblemSolver, set to 1 instead of 2

#plotFrame (r, 'frame_group', [0,0,0], 0.6)

# First parabolas: vmax = 8m/s,  mu = 1.2
cl.problem.setFrictionCoef(1.2); cl.problem.setMaxVelocityLim(4.5)
ps.clearRoadmap();
solveTime = ps.solve () # 0.312 s  388 nodes
pathId = ps.numberPaths()-offsetOrientedPath # path without orientation stuff

pathSamples = plotSampleSubPath (cl, r, pathId, 70, "path0", [0,0,1,1])
plotCone (q1, cl, r, "cone_first", "friction_cone_SG2"); plotCone (q2, cl, r, "cone_second", "friction_cone_SG2")
plotConeWaypoints (cl, pathId, r, "cone_wp_group", "friction_cone_WP2")

gui.writeNodeFile('cone_wp_group','cones_path.dae')
gui.writeNodeFile('cone_first','cone_start.dae')
gui.writeNodeFile('cone_second','cone_goal.dae')


# Second parabolas: vmax = 6.5m/s,  mu = 0.5
cl.problem.setFrictionCoef(0.5); cl.problem.setMaxVelocityLim(4.5)
ps.clearRoadmap();
solveTime = ps.solve () # 0.9689 s
pathId = ps.numberPaths()-offsetOrientedPath

pathSamples = plotSampleSubPath (cl, r, pathId, 70, "path1", [1,0,0,1])
plotCone (q1, cl, r, "cone_first", "friction_cone"); plotCone (q2, cl, r, "cone_second", "friction_cone_SG")
plotConeWaypoints (cl, pathId, r, "cone_wp_group", "friction_cone_WP")

gui.writeNodeFile('cone_wp_group','cones_path.dae')
gui.writeNodeFile('cone_first','cone_start.dae')
gui.writeNodeFile('cone_second','cone_goal.dae')


## 3D Plot tools ##

#gui.removeFromGroup("path0",r.sceneName)
#gui.getNodeList()
#gui.createGroup ('sphere_wp0_group')
#gui.getGroupNodeList ('sphere_wp0_group')


## IMPORT SCENE AND CONFIGS TO BLENDER ##
# I skip the "collada generation part" since I already have them from blender ...
blender/urdf_to_blender.py -p /local/mcampana/devel/hpp/videos/ -i /local/mcampana/devel/hpp/src/animals_description/urdf/sphere_mesh.urdf -o robot_blend.py # generate robot loadable by Blender

from viewer_display_library import pathToYamlFile, writeEdgeSamples, writePathSamples, writeSkipList

len(np.arange(0, ps.pathLength(pathId), 0.02))
pathToYamlFile (cl, r, "frames.yaml ", "robot/base_link", pathId, q2, 0.02)

ps.numberNodes()


# Plot cones and edges in viewer
plotConesRoadmap (cl, r, 'cone_rm_group', "friction_cone2")
plotEdgesRoadmap (cl, r, 'edgeGroup', 70, [0,1,0.2,1])

gui.writeNodeFile('cone_rm_group','cones_RM.dae')

## Write EDGES in a file, which will be parsed by a Blender-Python script
writeEdgeSamples (cl, 'edges.txt', 70)

## Write PATH samples in a file, which will be parsed by a Blender-Python script
pathSamples = plotSampleSubPath (cl, r, pathId, 70, "path0", [0,0,1,1])
writePathSamples (pathSamples, 'path.txt')

## Write RM edge and node index associated to solution-path:
writeSkipList (cl, 'indexes.txt') #[0, 3, 12, 16, 24, 21, 10, 8] #[0, 0, 6, 15, 17, 18, 13, 14]


