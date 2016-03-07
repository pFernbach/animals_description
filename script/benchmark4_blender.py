#/usr/bin/env python
# Benchmark4. Script which goes with animals_description package.
# The script launches a point-robot and the scene_jump_harder (mesh version 10) environment.
# It defines init and final configs, and solve them for .. couples of mu / vmax (parabola constraints).


from hpp.corbaserver.sphere import Robot
#from hpp.corbaserver.sphere_mesh import Robot
from hpp.corbaserver import Client
from hpp.corbaserver import ProblemSolver
from viewer_display_library import normalizeDir, plotCone, plotFrame, plotThetaPlane, shootNormPlot, plotStraightLine, plotConeWaypoints, plotSampleSubPath, contactPosition, addLight, plotSphere, plotSpheresWaypoints, plotConesRoadmap
import math
import numpy as np

robot = Robot ('robot')
robot.setJointBounds('base_joint_xyz', [-6, 6.8, -2.5, 3.2, 0, 8])
ps = ProblemSolver (robot)
cl = robot.client

# Configs : [x, y, z, q1, q2, q3, q4, dir.x, dir.y, dir.z, theta]
q11 = [6.2, 0.5, 0.5, 0, 0, 0, 1, 0, 0, 1, 0]; q22 = [-4.4, -1.5, 6.5, 0, 0, 0, 1, 0, 0, 1, 0]

from hpp.gepetto import Viewer, PathPlayer
r = Viewer (ps); gui = r.client.gui
pp = PathPlayer (robot.client, r)
r.loadObstacleModel ("animals_description","scene_jump_harder","scene_jump_harder")

q1 = cl.robot.projectOnObstacle (q11, 0.001); q2 = cl.robot.projectOnObstacle (q22, 0.001)
robot.isConfigValid(q1); robot.isConfigValid(q2)
r(q1)

ps.setInitialConfig (q1); ps.addGoalConfig (q2)

offsetOrientedPath = 2 # If remove oriented path computation in ProblemSolver, set to 1 instead of 2

#plotFrame (r, 'frame_group', [0,0,0], 0.6)

# First parabolas: vmax = 7m/s,  mu = 1.2
cl.problem.setFrictionCoef(1.2); cl.problem.setMaxVelocityLim(7)
ps.clearRoadmap();
solveTime = ps.solve () # 299 nodes
pathId = ps.numberPaths()-offsetOrientedPath # path without orientation stuff

pp.displayPath(pathId)
plotCone (q1, cl, r, "cone_first", "friction_cone2"); plotCone (q2, cl, r, "cone_second", "friction_cone2")
plotConeWaypoints (cl, pathId, r, "cone_wp_group", "friction_cone2")
#plotConesRoadmap (cl, r, 'cone_rm_group', "friction_cone2")

gui.writeNodeFile('cone_wp_group','cones_wp0_scene_jump_easy.dae')
gui.writeNodeFile('cone_first','cone_first_scene_jump_easy.dae')
gui.writeNodeFile('cone_second','cone_second_scene_jump_easy.dae')
gui.writeNodeFile('path_0_root','path_scene_jump_easy.obj')
#gui.writeNodeFile('cone_rm_group','cones_rm_scene_jump_easy.dae')

# Second parabolas: vmax = 6.5m/s,  mu = 0.5  # DO NOT SOLVE FIRST PATH BEFORE
cl.problem.setFrictionCoef(0.5); cl.problem.setMaxVelocityLim(6.5)
ps.clearRoadmap();
solveTime = ps.solve () # 4216 nodes .... 37848 edges
pathId = ps.numberPaths()-offsetOrientedPath

pp.displayPath(pathId)
plotCone (q1, cl, r, "cone_first", "friction_cone"); plotCone (q2, cl, r, "cone_second", "friction_cone")
plotConeWaypoints (cl, pathId, r, "cone_wp_group", "friction_cone")
#plotConesRoadmap (cl, r, 'cone_rm_group', "friction_cone")

gui.writeNodeFile('cone_wp_group','cones_wp0_scene_jump_hard.dae')
gui.writeNodeFile('cone_first','cone_first_scene_jump_hard.dae')
gui.writeNodeFile('cone_second','cone_second_scene_jump_hard.dae')
gui.writeNodeFile('path_0_root','path_scene_jump_hard.obj')
#gui.writeNodeFile('cone_rm_group','cones_rm_scene_jump_hard.dae')

## 3D viewer tools ##

#gui.removeFromGroup("path0",r.sceneName)
#gui.getNodeList()
#gui.createGroup ('sphere_wp0_group')
#gui.getGroupNodeList ('sphere_wp0_group')


## IMPORT SCENE AND CONFIGS TO BLENDER ##
# I skip the "collada generation part" since I already have them from blender ...
blender/urdf_to_blender.py -p /local/mcampana/devel/hpp/videos/ -i /local/mcampana/devel/hpp/src/animals_description/urdf/sphere_mesh.urdf -o robot_blend.py # generate robot loadable by Blender

pathId = 1
dt = 0.02
PL = ps.pathLength(pathId)
FrameRange = np.arange(0,PL,dt)
numberFrame = len(FrameRange)

gui.setCaptureTransform ("frames_scene_jump_easy1.yaml", ["robot/base_link"]) # DON'T FORGET TO RENAME
# Frame by Frame
for t in FrameRange:
        q = ps.configAtParam (pathId, t)#update robot configuration
        r (q); robot.setCurrentConfig(q)
        gui.refresh ()
        gui.captureTransform ()

r (q2); robot.setCurrentConfig(q2)
gui.refresh ()
gui.captureTransform ()


# Roadmap nodes and path .... #
# try Joseph stuff
gui.createRoadmap ('roadmap', [1,0,0,1], 0.03, 0.1, [0.5,0,0,1])
gui.addEdgeToRoadmap ('roadmap', posFrom, posTo)
gui.addNodeToRoadmap ('roadmap', q)

# Tentative for roadmap ....
len(ps.nodes())
plotConesRoadmap (cl, r, 'cone_rm_group', "friction_cone2")

gui.writeNodeFile('cone_rm_group','cones_rm_scene_jump_hard.dae')


