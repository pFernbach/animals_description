#/usr/bin/env python
# Script which goes with animals_description package.
# Functions library to display graphic things (trajectory, cone, frame...) on the Gepetto-viewer
# And functions to write data which will be parsed by Blender

from __future__ import division
import numpy as np
import math
#from hpp.gepetto import Viewer, PathPlayer

Pi = math.pi

# --------------------------------------------------------------------#

# Normalize the dir part of the configuration (dir = direction of cone)
## Parameters:
# q: given configuration
# robot: the robot
def normalizeDir (q, robot):
    q_out = q[::] # copy
    index = robot.getConfigSize () - 3
    N_norm = math.sqrt (q [index]**2 + q [index+1]**2 + q [index+2]**2)
    q_out [index] = q [index]/N_norm
    q_out [index+1] = q [index+1]/N_norm
    q_out [index+2] = q [index+2]/N_norm
    return q_out;

# --------------------------------------------------------------------#

## Plot whole path in viewer (in blue) WARNING: NOT ADAPTED FOR PARABOLA ##
# For parabola, prefrer method plotSampleSubPath
## Parameters:
# cl: corbaserver client
# nPath: path number
# r: viewer server
# lineNamePrefix: string prefix used for line name
# dt: step time
def plotPath (cl, nPath, r, lineNamePrefix, dt):
    for t in np.arange(0., cl.problem.pathLength(nPath), dt):
        lineName = lineNamePrefix+str(t)
        r.client.gui.addLine(lineName,[cl.problem.configAtParam(nPath, t)[0],cl.problem.configAtParam(nPath, t)[1],cl.problem.configAtParam(nPath, t)[2]],[cl.problem.configAtParam(nPath, t+dt)[0],cl.problem.configAtParam(nPath, t+dt)[1],cl.problem.configAtParam(nPath, t+dt)[2]],[0,0.3,1,1])
        r.client.gui.addToGroup (lineName, r.sceneName)

# --------------------------------------------------------------------#

## Plot global frame (in white, x darker) ##
## Parameters:
# r: viewer server
# frameGroupName: string used for group name, allowing to plot several frames
# framePosition: the [x, y, z] absolute position of the frame
# ampl: amplitude of the frame axis
def plotFrame (r, frameGroupName, framePosition, ampl):
    r.client.gui.createGroup (frameGroupName)
    x = framePosition [0]; y = framePosition [1]; z = framePosition [2];
    r.client.gui.addLine('frame1',[x,y,z], [x+ampl,y,z],[1,0,0,1])
    r.client.gui.addToGroup ('frame1', frameGroupName)
    r.client.gui.addLine('frame2',[x,y,z], [x,y+ampl,z],[0,1,0,1])
    r.client.gui.addToGroup ("frame2", frameGroupName)
    r.client.gui.addLine('frame3',[x,y,z], [x,y,z+ampl],[0,0,1,1])
    r.client.gui.addToGroup ('frame3', frameGroupName)
    r.client.gui.addSceneToWindow(frameGroupName,r.windowId)
    #r.client.gui.removeFromGroup('frame1',r.sceneName) # remove duplicata in sceneName NOT WORKING
    #r.client.gui.removeFromGroup('frame2',r.sceneName)
    #r.client.gui.removeFromGroup('frame3',r.sceneName)

# --------------------------------------------------------------------#

## Plot cone at each waypoint of the path ##
## Parameters:
# cl: corbaserver client
# nPath: path number
# r: viewer server
# coneGroupName: string used for cone group name ('cone_wp_group')
# coneURDFname: "friction_cone" (mu = 0.5) or "friction_cone2" (mu = 1.2)
# To avoid problem with cone names in Blender, use also "friction_cone_WP"...
def plotConeWaypoints (cl, nPath, r, coneGroupName, coneURDFname):
    wp = cl.problem.getWaypoints (nPath)
    r.client.gui.createGroup (coneGroupName)
    for i in np.arange(1, len(wp)-1, 1): # avoid (re-)plot start and goal
        coneName = coneGroupName+'_'+"cone_"+str(i)
        plotCone (wp[i], cl, r, coneName, coneURDFname)
        r.client.gui.addToGroup (coneName, coneGroupName)
    r.client.gui.addSceneToWindow(coneGroupName,r.windowId)


# --------------------------------------------------------------------#

## Plot cone ##
## Parameters:
# cl: corbaserver client
# q: configuration of cone (position, orientation)
# r: viewer server
# coneName: string used for cone name (e.g. "cone_wp0/this_cone")
# coneURDFname: "friction_cone" (mu = 0.5) or "friction_cone2" (mu = 1.2)
# To avoid problem with cone names in Blender, use also "friction_cone_SG"...
def plotCone (q, cl, r, coneName, coneURDFname):
    qCone = cl.robot.setOrientation (q)
    r.loadObstacleModel ("animals_description",coneURDFname,coneName)
    r.client.gui.applyConfiguration (coneName, qCone[0:7])
    r.client.gui.refresh ()
#Could be:
#gui.addMesh (coneName,"/local/mcampana/devel/hpp/src/animals_description/meshes/cone2.dae")
#gui.applyConfiguration (coneName, q11[0:7]); gui.addToGroup (coneName, r.sceneName)
#gui.refresh ()
# --------------------------------------------------------------------#

## Plot straight line ##
# Uses: plot cone direction or cone - plane_theta intersection
## Parameters:
# vector: direction vector
# pos: origin-position of straight line
# r: viewer server
# lineNamePrefix: string prefix used for line name
def plotStraightLine (vector, pos, r, lineNamePrefix):
    x0 = pos[0]
    y0 = pos[1]
    z0 = pos[2]
    x = vector[0]
    y = vector[1]
    z = vector[2]
    lineName=lineNamePrefix+"straight"
    r.client.gui.addLine(lineName,[x0,y0,z0], [x0+x,y0+y,z0+z],[1,0.3,0.3,1])
    r.client.gui.addToGroup (lineName, r.sceneName)

# --------------------------------------------------------------------#

## Plot plane theta ##
# (with two triangles, last pos not taken in addSquareFace)
## Parameters:
# q1: initial configuration
# q2: final configuration
# r : viewer server
# lineNamePrefix: string prefix used for line name
def plotThetaPlane (q1, q2, r, lineNamePrefix):
    pos1 = [q1 [0], q1 [1], q1 [2]+5]
    pos2 = [q2 [0], q2 [1], q2 [2]-5]
    pos3 = [q1 [0], q1 [1], q1 [2]-5]
    pos4 = [q2 [0], q2 [1], q2 [2]+5]
    r.client.gui.addSquareFace (lineNamePrefix, pos1, pos2, pos4, pos4, [0.7,0.2,0.2,0.5])
    r.client.gui.addToGroup (lineNamePrefix, r.sceneName)
    lineNamePrefix_bis = lineNamePrefix+"bis"
    r.client.gui.addSquareFace (lineNamePrefix_bis, pos1, pos3, pos2, pos2, [0.7,0.2,0.2,0.5])
    r.client.gui.addToGroup (lineNamePrefix_bis, r.sceneName)

# --------------------------------------------------------------------#

## Shoot random config, normalize 'dir' part and plot cone ##
## Parameters:
# cl: corbaserver client
# r: viewer server
# mu: cone coefficient of friction
# ampl: cone amplitude
# lineNamePrefix: string prefix used for line name
def shootNormPlot (cl, r, mu, ampl, lineNamePrefix):
    q = cl.robot.shootRandomConfig ()
    print q
    q = normalizeDir (q, cl.robot)
    plotCone (q, cl, r, mu, ampl, lineNamePrefix)
    index = cl.robot.getConfigSize () - 4
    plotStraightLine ([q [index],q [index+1],q [index+2]], q, r, lineNamePrefix+"normale")
    return q

# --------------------------------------------------------------------#

## Plot path choosing the number of samples by subpath ##
## Parameters:
# cl: corbaserver client
# r: viewer server
# nPath: path number
# NbPointsPerSubPath: number of sampled points per subpath (each parabola)
# curvePrefix: string prefix used for curve name
# curveColor: osg-color of the curve (e.g. [0,1,0,1])
def plotSampleSubPath (cl, r, nPath, NbPointsPerSubPath, curvePrefix, curveColor):
    plotSampleConfigs = cl.problem.sampleSubPath(nPath, NbPointsPerSubPath)
    pointsCurv = []
    for i in range(0, len(plotSampleConfigs)):
        pointsCurv.append ([plotSampleConfigs [i][0], plotSampleConfigs [i][1], plotSampleConfigs [i][2]])
    
    r.client.gui.addCurve (curvePrefix, pointsCurv, curveColor)
    r.client.gui.addToGroup (curvePrefix, r.sceneName)
    return plotSampleConfigs

# --------------------------------------------------------------------#

## Return contact position (projected on nearest obstacle) ##
# WARNING: assuming that q has been projected and contains "good" orientation and normal
# (if one wants to plot the cone at the contact...)
## Parameters:
# q: CoM configuration (not at the contact)
# cl: corbaserver client
# r: viewer server
def contactPosition (q, cl, r):
    qConeContact = q[::] # at least for orientation
    index = cl.robot.getConfigSize () - cl.robot.getExtraConfigSize ()
    n = np.array([q[index], q[index+1], q[index+2]]) # normal
    cl.robot.setCurrentConfig (q)
    res = cl.robot.distancesToCollision ()
    pCoM = q[0:3]
    pj = res[4][np.argmin(res[0])] # point on obstacle surface
    distContactCoM = np.dot(np.array(pCoM)-np.array(pj),n)
    qConeContact[0:3] = (np.array(q[0:3]) - distContactCoM*n).tolist ()
    return qConeContact

# --------------------------------------------------------------------#

## Add light in viewer scene ##
# giving the light configuration (position) and name
# "r.client.gui.removeFromGroup (lightName, r.sceneName)" to remove light 
# (if think it just removes the object carrying the light, not the light effect)
## Parameters:
# r: viewer server
# q: light configuration (list)
# lightName: light name (string)
def addLight (r, q, lightName):
    r.client.gui.addLight (lightName, r.windowId, 0.0001, [0.9,0.9,0.9,1])
    r.client.gui.addToGroup (lightName, r.sceneName)
    r.client.gui.applyConfiguration (lightName, q)
    r.client.gui.refresh ()

# --------------------------------------------------------------------#

## Plot sphere ##
# Example (plot small green sphere) "plotSphere (q, cl, r, sphereName, [0,1,0,1], 0.02)"
## Parameters:
# cl: corbaserver client
# q: configuration of cone (position, orientation)
# r: viewer server
# sphereName: string suffix used for cone name
# sphereColor: color of sphere
# sphereSize: size of sphere
def plotSphere (q, cl, r, sphereName, sphereColor, sphereSize):
    r.client.gui.addSphere (sphereName,sphereSize,sphereColor)
    r.client.gui.applyConfiguration (sphereName, q[0:7])
    r.client.gui.addToGroup (sphereName, r.sceneName)
    r.client.gui.refresh ()

# --------------------------------------------------------------------#

## Plot sphere at each waypoint of the path ##
## Parameters:
# cl: corbaserver client
# nPath: path number
# r: viewer server
# sphereGroupName: string used for sphere group name
# sphereColor: color of sphere
# sphereSize: size of sphere
def plotSpheresWaypoints (cl, nPath, r, sphereGroupName, sphereColor, sphereSize):
    wp = cl.problem.getWaypoints (nPath)
    r.client.gui.createGroup (sphereGroupName)
    for i in np.arange(1, len(wp)-1, 1): # avoid (re-)plot start and goal
        sphereName = sphereGroupName+'_'+"sphere_"+str(i)
        plotSphere (wp[i][0:7], cl, r, sphereName, sphereColor, sphereSize)
        r.client.gui.addToGroup (sphereName, sphereGroupName)
        #r.client.gui.removeFromGroup(sphereName,r.sceneName) # remove duplicata in sceneName NOT WORKING
    r.client.gui.addSceneToWindow(sphereGroupName,r.windowId)

# --------------------------------------------------------------------#

## Plot cone at each node of the roadmap ##
## Parameters:
# cl: corbaserver client
# r: viewer server
# coneGroupName: string used for cone group name
# coneURDFname: "friction_cone" (mu = 0.5) or "friction_cone2" (mu = 1.2)
def plotConesRoadmap (cl, r, coneGroupName, coneURDFname):
    RM_nodes = cl.problem.nodes ()
    r.client.gui.createGroup (coneGroupName)
    for i in range(2,len(RM_nodes)): # avoid first nodes
        node_i = RM_nodes [i]
        coneName_i = "cone_RM_"+str(i)
        plotCone (node_i, cl, r, coneName_i, coneURDFname)
        r.client.gui.addToGroup (coneName_i, coneGroupName)
    r.client.gui.addSceneToWindow(coneGroupName,r.windowId)

# --------------------------------------------------------------------#

## Plot each edge of the roadmap from sampled points ##
## Parameters:
# cl: corbaserver client
# r: viewer server
# edgeGroupName: edge group name
def plotEdgesRoadmap (cl, r, edgeGroupName, nbPointsPerEdge, curveColor):
    gui = r.client.gui
    gui.createGroup (edgeGroupName)
    numEdges = cl.problem.numberEdges ()
    for k in range (0,numEdges,2): # one over two (avoid reverse edge)
        samplesEdge_k = cl.problem.edgeToSampledPathVector (k, nbPointsPerEdge)
        pointsEdge_k = []
        for i in range(0, len(samplesEdge_k)):
            pointsEdge_k.append ([samplesEdge_k [i][0], samplesEdge_k [i][1], samplesEdge_k [i][2]])
        nameEdge_k = edgeGroupName+'edge_'+str(k)
        r.client.gui.addCurve (nameEdge_k, pointsEdge_k, curveColor)
        r.client.gui.addToGroup (nameEdge_k, edgeGroupName)
    
    r.client.gui.addSceneToWindow(edgeGroupName,r.windowId)

# --------------------------------------------------------------------#

# ----------------------------## BLENDER ##------------------------------------#

## Write Path-motion in Yaml file ##
## Parameters:
# cl: corbaserver client
# fileName: name (string) of the file where samples will be written
def pathToYamlFile (cl, r, fileName, robotName, pathId, goalConfig, dt):
    gui = r.client.gui
    FrameRange = np.arange(0, cl.problem.pathLength(pathId), dt)
    gui.setCaptureTransform (fileName, [robotName])
    for t in FrameRange:
        q = cl.problem.configAtParam (pathId, t)#update robot configuration
        r (q); cl.robot.setCurrentConfig(q)
        gui.refresh ()
        gui.captureTransform ()
    
    r (goalConfig); cl.robot.setCurrentConfig(goalConfig)
    gui.refresh ()
    gui.captureTransform ()

# --------------------------------------------------------------------#

## Write Edge samples to text file ##
# because exporting the edges directly in .obj creates BIG files
## Parameters:
# cl: corbaserver client
# fileName: name (string) of the file where samples will be written
def writeEdgeSamples (cl, fileName, nbPointsPerEdge):
    numEdges = cl.problem.numberEdges ()
    f = open(fileName,'a')
    for k in range (0,numEdges,2):
        print ("Edge number: " + str(k))
        samplesEdge_k = cl.problem.edgeToSampledPathVector (k, nbPointsPerEdge)
        f.write('e'+'\n')
        for i in range(0, len(samplesEdge_k)):
            f.write(str(samplesEdge_k [i]).strip('[]')+'\n') # write point i
    
    f.close()

# --------------------------------------------------------------------#

## Write Path samples to text file ##
# require samples from "pathSamples = plotSampleSubPath (cl, r, pathId, 70, "path0", [0,0,1,1])"
## Parameters:
# pathSamples: sample used to plot solution-path in viewer
# fileName: name (string) of the file where samples will be written
def writePathSamples (pathSamples, fileName):
    numPathSamples = len(pathSamples)
    f = open(fileName,'a')
    for i in range (0,numPathSamples):
        print ("sample number: " + str(i))
        f.write(str(pathSamples [i]).strip('[]')+'\n') # write point i
    
    f.close()

# --------------------------------------------------------------------#

## Write RM edge and node index associated to solution-path:
# cl: corbaserver client
# fileName: name (string) of the file where samples will be written
def writeSkipList (cl, fileName):
    l1 = cl.problem.getEdgeIndexVector() #[0, 3, 12, 16, 24, 21, 10, 8]
    l2 = cl.problem.getNodeIndexVector() #[0, 0, 6, 15, 17, 18, 13, 14]
    del l1 [0]; del l2 [0]; del l2 [0];
    f = open(fileName,'a')
    f.write(str(l1).strip('[]')+'\n')
    f.write(str(l2).strip('[]')+'\n')
    f.close()

