#/usr/bin/env python
# RUNS version! launch command: "python -i scene_jump_harder_runs.py"
# Script which goes with animals_description package, runs version of Benchmark4.
# The script launches a point-robot and the scene_jump environment.
# It defines init and final configs, and solve them for .. couples of mu / vmax (parabola constraints).
# Do not forget to launch it in Release mode !

from hpp.corbaserver.sphere import Robot
from hpp.corbaserver import Client
from hpp.corbaserver import ProblemSolver
import math
import numpy as np

robot = Robot ('robot')
robot.setJointBounds('base_joint_xyz', [-6, 6.8, -2.5, 3.2, 0, 8]) # ultimate goal!
ps = ProblemSolver (robot)
cl = robot.client
cl.obstacle.loadObstacleModel('animals_description','scene_jump_harder','')

# Configs : [x, y, z, q1, q2, q3, q4, dir.x, dir.y, dir.z, theta]
q11 = [6.2, 0.5, 0.5, 0, 0, 0, 1, 0, 0, 1, 0]; q22 = [-4.4, -1.5, 6.5, 0, 0, 0, 1, 0, 0, 1, 0] # start to ultimate goal!

q1 = cl.robot.projectOnObstacle (q11, 0.001); q2 = cl.robot.projectOnObstacle (q22, 0.001)
ps.setInitialConfig (q1); ps.addGoalConfig (q2)

#vmax = 7.0; mu = 1.2 # results1.txt
#vmax = 7.0; mu = 0.5 # results2.txt
#vmax = 6.5; mu = 1.2 # results3.txt
vmax = 6.5; mu = 0.5
cl.problem.setFrictionCoef(mu); cl.problem.setMaxVelocityLim(vmax)

toSeconds = np.array ([60*60,60,1,1e-3])
offsetOrientedPath = 2 # If remove oriented path computation in ProblemSolver, set to 1 instead of 2
imax=40;
f = open('results.txt','a')
    
# Assuming that seed in modified directly in HPP (e.g. in core::PathPlanner::solve or ProblemSolver constr)
for i in range(0, imax):
    print i
    ps.clearRoadmap ()
    solveTimeVector = ps.solve ()
    solveTime = np.array (solveTimeVector).dot (toSeconds)
    pathId = ps.numberPaths()-offsetOrientedPath
    pathLength = ps.pathLength (pathId)
    pathNumberWaypoints = len(ps.getWaypoints (pathId))
    roadmapNumberNodes = ps.numberNodes ()
    results = cl.problem.getResultValues ()
    failCollisionNumber = results [0]
    failIntersectionNumber = results [1]
    failConstraintNumber = results [2]
    
    #ps.addPathOptimizer("Prune")
    #ps.optimizePath (pathId)
    #prunePathId = ps.numberPaths()-1
    
    # Write important results #
    f.write('Try number: '+str(i)+'\n')
    f.write('with parameters: vmax='+str(vmax)+' and mu='+str(mu)+'\n')
    f.write('solve duration: '+str(solveTime)+'\n')
    f.write('path length: '+str(pathLength)+'\n')
    f.write('number of waypoints: '+str(pathNumberWaypoints)+'\n')
    f.write('number of nodes: '+str(roadmapNumberNodes)+'\n')
    f.write('number of fails due to collision: '+str(failCollisionNumber)+'\n')
    f.write('number of fails due to cone intersection: '+str(failIntersectionNumber)+'\n')
    f.write('number of fails due to constraints: '+str(failConstraintNumber)+'\n')

f.close()

from parseRunsParabola import main
main()


