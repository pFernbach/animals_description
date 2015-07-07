#/usr/bin/env python
# Author : Mylene Campana
# Script to plot some graphs after launching a corbaserver and solving 
# the problem. The main DIFFERENCE here is that the plot is always returned so that 
# plots can be added later in the Python interface.
# User has to call himself "plt.show()"

#import matplotlib.pyplot as plt
from __future__ import division
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import math

dt = 0.02 # global drawing step size
Pi = math.pi

# --------------------------------------------------------------------#

def parabPlot (cl, nPath, mu, plt):
    plt = plotRectangles_parab (plt) # plot environment
    #plt = plotHardRectangles_parab (plt) # plot harder environment
    init = cl.problem.getInitialConfig ()
    goal = cl.problem.getGoalConfigs ()[0] # first goal
    
    # Plot whole path
    for t in np.arange(0., cl.problem.pathLength(nPath), dt):
        plt.plot([cl.problem.configAtParam(nPath, t)[0], \
                     cl.problem.configAtParam(nPath, t+dt)[0]], \
                     [cl.problem.configAtParam(nPath, t)[1], \
                     cl.problem.configAtParam(nPath, t+dt)[1]], 'k', linewidth=1.8)
    
    # Plot cones at each waypoint
    wp = cl.problem.getWaypoints (nPath)
    ampl = 0.5 # cone amplitude
    for i in np.arange(0, len(wp), 1):
        x = wp[i][0]
        y = wp[i][1]
        gamma = math.atan2(wp[i][3], wp[i][2]) - Pi/2;
        alpha_min = Pi/2 + gamma - math.atan(mu)
        alpha_max = Pi/2 + gamma + math.atan(mu)
        plt.plot([x, x+ampl*math.cos(alpha_min)],[y, y+ampl*math.sin(alpha_min)], 'm', linewidth=1.1)
        plt.plot([x, x+ampl*math.cos(alpha_max)],[y, y+ampl*math.sin(alpha_max)], 'm', linewidth=1.1)
    
    plt.legend()
    plt.axis([-2, 13, -6, 7])
    plt.xlabel('x'); plt.ylabel('y')
    #plt.title('parabola planning')
    plt.grid()
    plt.plot(init[0], init[1], 'go')
    plt.plot(goal[0], goal[1], 'go')
    #plt.text(init[0]+.1, init[1]+.1, r'q_init', fontsize=11)
    #plt.text(goal[0]+.1, goal[1]+.1, r'q_end', fontsize=11)
    return plt



# --------------------------------------------------------------------#

# Plot 2D nodes (from parseLog) with given color and text.
# For example, nodeName = r'qCol'    and   nodeColor = 'bo'
def addNodePlot (nodeList, nodeColor, nodeName, plt):
    i = 0
    for n in nodeList :
        plt.plot(n[0], n[1], nodeColor)
        #plt.text(n[0]+.02, n[1], nodeName+'%i' %(i), fontsize=8)
        i = i+1
    return plt


# --------------------------------------------------------------------#

# Plot a parsed parabola
def plotParsedParab (parab, color, lw, plt):
    for i in np.arange(0, len(parab)-1, 1):
        plt.plot([parab[i][0], parab[i+1][0]],[parab[i][1], parab[i+1][1]], color, linewidth=lw)
    return plt

# --------------------------------------------------------------------#

# Plot 2D rectangles for parabola environment
def plotRectangles_parab (plt):
    obstacles = [[-4,6],[-4,2],[-2,0],[0,0],[4,-4],[4,-3],[6,-3],[8,-1],[8,2],[10,2],[10,-6],[13,-6],[13,6]];
    obstacles = np.array (obstacles)
    lw = 5.5 # linewidth
    obstColor = 'r'
    for i in np.arange(0, len(obstacles)-1, 1):
        plt.plot([obstacles[i][0], obstacles[i+1][0]], [obstacles[i][1], obstacles[i+1][1]], obstColor, linewidth=lw)
    return plt

# --------------------------------------------------------------------#

# Plot 2D rectangles for parabola harder environment
def plotHardRectangles_parab (plt):
    lw = 2 # linewidth
    obstColor = 'r'
    obstacles = [[-4,6],[-4,2],[-2,0],[0,0],[4,-4],[4,-3],[6,-3],[8,-1],[8,2],[10,2],[10,-6],[13,-6],[13,-4],[11,-4],[11,-3],[13,-3],[13,6]];
    obstacles = np.array (obstacles)
    
    obstacles1 = [[10,0],[11,0],[11,1],[10,1],[10,0]] # additional obst
    obstacles1 = np.array (obstacles1)
    
    obstacles2 = [[0,2],[1,2],[1,3],[0,3],[0,2]] # additional obst
    obstacles2 = np.array (obstacles2)
    
    obstacles3 = [[4,3],[5,3],[5,4],[4,4],[4,3]] # additional obst
    obstacles3 = np.array (obstacles3)
    
    for i in np.arange(0, len(obstacles)-1, 1):
        plt.plot([obstacles[i][0], obstacles[i+1][0]], [obstacles[i][1], obstacles[i+1][1]], obstColor, linewidth=lw)
    for i in np.arange(0, len(obstacles1)-1, 1):
        plt.plot([obstacles1[i][0], obstacles1[i+1][0]], [obstacles1[i][1], obstacles1[i+1][1]], obstColor, linewidth=lw)
    for i in np.arange(0, len(obstacles2)-1, 1):
        plt.plot([obstacles2[i][0], obstacles2[i+1][0]], [obstacles2[i][1], obstacles2[i+1][1]], obstColor, linewidth=lw)
    for i in np.arange(0, len(obstacles3)-1, 1):
        plt.plot([obstacles3[i][0], obstacles3[i+1][0]], [obstacles3[i][1], obstacles3[i+1][1]], obstColor, linewidth=lw)
    return plt

