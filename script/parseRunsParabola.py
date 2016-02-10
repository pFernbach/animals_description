#/usr/bin/env python

# Parser script for run tests results
# Average and Variance of parsed values :
# path durations, number of 'new' nodes, computation time

"""
from parseRunsParabola import main
main()
"""

from __future__ import division
import numpy as np
import math

runFile = "/local/mcampana/devel/hpp/src/animals_description/script/"
fileName = "results.txt"

# --------------------------------------------------------------------#

def main ():
    solveDurationPrefix = 'solve duration: '
    pathLengthPrefix = 'path length: '
    numberWaypointsPrefix = 'number of waypoints: '
    numberNodesPrefix = 'number of nodes: '
    
    ## Compute data:
    solveDurationVector = parseRun (solveDurationPrefix)
    pathlengthVector = parseRun (pathLengthPrefix)
    numberWaypointsVector = parseRun (numberWaypointsPrefix)
    numberNodesVector = parseRun (numberNodesPrefix)
    
    ## Display data:
    print "Number of data: " + str(len (solveDurationVector))+'\n'

    print "solve duration"
    print "mean: " + str(np.mean(solveDurationVector))
    print "std: " + str(np.std(solveDurationVector))+'\n'
    
    print "path length"
    print "mean: " + str(np.mean(pathlengthVector))
    print "std: " + str(np.std(pathlengthVector))+'\n'
    
    print "number of waypoints"
    print "mean: " + str(np.mean(numberWaypointsVector))
    print "std: " + str(np.std(numberWaypointsVector))+'\n'
    
    print "number of nodes"
    print "mean: " + str(np.mean(numberNodesVector))
    print "std: " + str(np.std(numberNodesVector))+'\n'


# --------------------------------------------------------------------#

def parseRun (prefix):
    l = len (prefix)
    with open (runFile + fileName) as f:
        vector = []
        for line in f:
            if line [:l] == prefix :
                elem = line [l:]
                st = elem.strip ('\n') # remove end characters
                try:
                    #config = map (float, elem) # convert into float
                    vector.append (float(st))
                except:
                    print "catched"
    return np.array (vector)

# --------------------------------------------------------------------#

# Compute average of a vector
def average (vector):
    av=0
    for elem in vector:
        av += elem
    av = av/len(vector)
    print "average: "+str(av)
    return av

# --------------------------------------------------------------------#

# Compute variance of a vector (Konig-Huygens formula)
def variance (vector):
    av = average (vector)
    var=0
    for elem in vector:
        var += elem**2
    var = var/len(vector)
    var = var - av**2
    if var<0:
        print "Variance is <0 because of rounding errors: "+str(var)
        var=0
    return var

# --------------------------------------------------------------------#

# Deprecated
# Compute average and SD for the given inString indicating a vector of floats in 'results.txt'
def compute (inString):
    vector = parseRun (inString)
    print "Number of data: " + str(len (vector))
    #print vector
    print "SD: "+str(math.sqrt(variance (vector)))+"\n"

# --------------------------------------------------------------------#

# Compute distance of the base motion (x-y) along a path
# wps: is the list of waypoints of the path
def computeBaseMotion (wps):
    dist = 0
    for i in range(0,len(wps)-1):
        wpi = wps [i]
        wpii = wps [i+1]
        dist += math.sqrt( (wpii [0] - wpi [0])**2 +  (wpii [1] - wpi [1])**2  )
    return dist

