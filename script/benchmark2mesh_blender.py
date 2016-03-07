#/usr/bin/env python
# Benchmark2mesh. Script which goes with animals_description package.
# The script launches a point-robot and the environment containing an environment 3d mesh (with windows).
# It defines init and final configs, and solve them for 2 couples of mu / vmax (parabola constraints).

#blender/urdf_to_blender.py -p /local/mcampana/devel/hpp/videos/ -i robot.urdf -o robot_blend.py

from hpp.corbaserver.sphere import Robot
from hpp.corbaserver import Client
from hpp.corbaserver import ProblemSolver
from viewer_display_library import normalizeDir, plotCone, plotFrame, plotThetaPlane, shootNormPlot, plotStraightLine, plotConeWaypoints, plotSampleSubPath, contactPosition, addLight, plotSphere, plotSpheresWaypoints, plotConesRoadmap
import math
import numpy as np

robot = Robot ('robot')
robot.setJointBounds('base_joint_xyz', [-5, 5, -5, 5, -1, 7])
ps = ProblemSolver (robot)
cl = robot.client

# Configs : [x, y, z, q1, q2, q3, q4, dir.x, dir.y, dir.z, theta]
q11 = [-3.8, 2.4, 1.2, 0, 0, 0, 1, 0, 0, 1, 0]; q22 = [3.5, -3.3, 0.4, 0, 0, 0, 1, 0, 0, 1, 0]

from hpp.gepetto import Viewer, PathPlayer
r = Viewer (ps); gui = r.client.gui
pp = PathPlayer (robot.client, r)
r.loadObstacleModel ("animals_description","envir3d_window_mesh","envir3d_window_mesh")
#addLight (r, [-3,3,4,1,0,0,0], "li"); addLight (r, [3,-3,4,1,0,0,0], "li1")
r(q11)

q1 = cl.robot.projectOnObstacle (q11, 0.001); q2 = cl.robot.projectOnObstacle (q22, 0.001)
robot.isConfigValid(q1); robot.isConfigValid(q2)
r(q1)

ps.setInitialConfig (q1); ps.addGoalConfig (q2)

offsetOrientedPath = 2 # If remove oriented path computation in ProblemSolver, set to 1 instead of 2

#plotFrame (r, 'frame_group', [0,0,0], 0.6)

# First parabola(s): vmax = 6.8m/s,  mu = 1.2
cl.problem.setFrictionCoef(1.2); cl.problem.setMaxVelocityLim(6.8)
ps.clearRoadmap();
solveTime = ps.solve () # 0.085 s
pathId = ps.numberPaths()-offsetOrientedPath # path without orientation stuff

pp.displayPath(pathId)
plotCone (q1, cl, r, "cone_first", "friction_cone2"); plotCone (q2, cl, r, "cone_second", "friction_cone2")
plotConeWaypoints (cl, pathId, r, "cone_wp_group", "friction_cone2")

gui.writeNodeFile('cone_wp_group','cones_wp0_envir3d_easy.dae')
gui.writeNodeFile('cone_first','cone_first_envir3d_easy.dae')
gui.writeNodeFile('cone_second','cone_second_envir3d_easy.dae')
gui.writeNodeFile('path_0_root','path_envir3d_easy.obj')


# Second parabola(s): vmax = 5.3m/s,  mu = 0.5
cl.problem.setFrictionCoef(0.5); cl.problem.setMaxVelocityLim(5.3)
ps.clearRoadmap();
solveTime = ps.solve () # 0.738 s  # 340 nodes
pathId = ps.numberPaths()-offsetOrientedPath

pp.displayPath(pathId)
plotCone (q1, cl, r, "cone_first", "friction_cone"); plotCone (q2, cl, r, "cone_second", "friction_cone")
plotConeWaypoints (cl, pathId, r, "cone_wp_group", "friction_cone")

gui.writeNodeFile('cone_wp_group','cones_wp0_envir3d_hard.dae')
gui.writeNodeFile('cone_first','cone_first_envir3d_hard.dae')
gui.writeNodeFile('cone_second','cone_second_envir3d_hard.dae')
gui.writeNodeFile('path_2_root','path_envir3d_hard.obj')



## 3D viewer tools ##

#gui.removeFromGroup("path0",r.sceneName)
#gui.getNodeList()
#gui.createGroup ('sphere_wp0_group')
#gui.getGroupNodeList ('sphere_wp0_group')
#ffmpeg -i untitled.mp4 -vcodec libx264 -crf 24 video.mp4  # to compress a video

## IMPORT SCENE AND CONFIGS TO BLENDER ##
blender/urdf_to_blender.py -p /local/mcampana/devel/hpp/videos/ -i /local/mcampana/devel/hpp/src/animals_description/urdf/sphere_mesh.urdf -o robot_blend.py # generate robot loadable by Blender

dt = 0.02
PL = ps.pathLength(pathId)
FrameRange = np.arange(0,PL,dt)
numberFrame = len(FrameRange)

gui.setCaptureTransform ("frames_envir3d_hard.yaml", ["robot/base_link"]) # DON'T FORGET TO RENAME

for t in FrameRange:
        q = ps.configAtParam (pathId, t)#update robot configuration
        r (q); robot.setCurrentConfig(q)
        gui.refresh ()
        gui.captureTransform ()

r (q2); robot.setCurrentConfig(q2)
gui.refresh ()
gui.captureTransform ()

# Tentative for roadmap ....
ps.numberNodes()
plotConesRoadmap (cl, r, 'cone_rm_group', "friction_cone")

gui.writeNodeFile('cone_rm_group','cones_rm_envir3d_easy1.dae')

#from euler_quat_math import Quaternion, test_q2e
node_blender_k = []
f = open('nodes.txt','a')
for k in range (0,ps.numberNodes()):
    print ("Node number: " + str(k))
    node_k = ps.nodes() [k]
    node_blender_k = node_k [0:7]
    #euler = test_q2e (Quaternion(node_k [3],node_k [4],node_k [5],node_k [6])) # in degrees
    #node_blender_k [3] = euler.x; node_blender_k [4] = euler.y; node_blender_k [5] = euler.z
    f.write(str(node_blender_k).strip('[]')+'\n') # write node k

f.close()

bpy.ops.wm.collada_import ("/local/mcampana/devel/hpp/src/animals_description/meshes/cone2.dae")


# Plot edges
edgeGroupName = 'edgeGroupThree'
gui.createGroup (edgeGroupName)
curveColor = [0,1,0.2,1]
#numEdges = ps.numberEdges ()
numEdges = 10
for k in range (0,numEdges,2): # one over two (avoid reverse edge)
    samplesEdge_k = cl.problem.edgeToSampledPathVector (k, 70)
    pointsEdge_k = []
    for i in range(0, len(samplesEdge_k)):
        pointsEdge_k.append ([samplesEdge_k [i][0], samplesEdge_k [i][1], samplesEdge_k [i][2]])
    nameEdge_k = edgeGroupName+"edge_"+str(k)
    r.client.gui.addCurve (nameEdge_k, pointsEdge_k, curveColor)
    r.client.gui.addToGroup (nameEdge_k, edgeGroupName)

r.client.gui.addSceneToWindow(edgeGroupName,r.windowId)

gui.writeNodeFile(edgeGroupName,'edgeGroupThree_envir3d_easy.obj') # 140 MB ...


## Write EDGES in a file, which will be parsed by a Blender-Python script
#samplesTest = [[[0,0,0],[1,1,0],[1,0,0],[-1,0,0]],[[0,0,1],[1,0,1],[0,1,1],[-1,0,1]]]
#numEdges = len(samplesTest)
numEdges = ps.numberEdges ()
f = open('edges.txt','a')
for k in range (0,numEdges,2):
    print ("Edge number: " + str(k))
    samplesEdge_k = cl.problem.edgeToSampledPathVector (k, 70)
    #samplesEdge_k	= samplesTest [k]
    f.write('e'+'\n')
    for i in range(0, len(samplesEdge_k)):
        f.write(str(samplesEdge_k [i]).strip('[]')+'\n') # write point i

f.close()


