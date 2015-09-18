#/usr/bin/env python
# Script which goes with animals_description package.
# Debug with freeflyer-box object

from hpp.corbaserver.box import Robot
from hpp.corbaserver import Client
from hpp.corbaserver import ProblemSolver
import numpy as np

robot = Robot ('robot')
ps = ProblemSolver (robot)
cl = robot.client

from hpp.gepetto import Viewer, PathPlayer
r = Viewer (ps)
pp = PathPlayer (robot.client, r)
r.loadObstacleModel ("room_description","room","room")

q1 = [1.4,-2.7,1.1,1,0,0,0]
r(q1)



r([1.48,-2.635,1.393,1,0,0,0])

r([1.48,-2.545,1.76,1,0,0,0])

r([0.7,-2.545,1.76,1,0,0,0])

r([0.7,-2.635,1.393,1,0,0,0])
