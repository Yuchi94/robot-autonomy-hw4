import numpy
import matplotlib.pyplot as pl
from random import *
import math
import time

import pdb
class HerbEnvironment(object):
    
    def __init__(self, herb):
        self.robot = herb.robot
        self.herb = herb
        # add a table and move the robot into place
        '''
        self.table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        #self.robot.GetEnv().Add(self.table)

        table_pose = numpy.array([[ 0, 0, -1, 0.6], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        self.table.SetTransform(table_pose)
        '''
        # set the camera
        camera_pose = numpy.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)
        
        # goal sampling probability
        self.p = 0.0

    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p
        
    def checkCollision(self, config):
        self.robot.SetActiveDOFValues(config)
	table = self.robot.GetEnv().GetBodies()[1]
        return self.robot.GetEnv().CheckCollision(self.robot, table)

    def GenerateRandomConfiguration(self):
        lower_limits, upper_limits = self.robot.GetActiveDOFLimits()
        while True:
            config = numpy.random.uniform(lower_limits, upper_limits, len(self.robot.GetActiveDOFIndices()))
            if not self.checkCollision(config):
                break

        return numpy.array(config)

    def GenerateRandomConfiguration_GoalBias(self, goal_config = None):
        
        if goal_config is None:
            goal = self.goal_config
        else:
            goal = goal_config

        if uniform(0,1) < self.p:
            return goal
        else:
            return self.GenerateRandomConfiguration()
    
    def ComputeDistance(self, start_config, end_config):
        
        return numpy.linalg.norm(numpy.array(start_config)-numpy.array(end_config))


    def Extend(self, start_config, end_config, max_extend):

        end_config = (end_config - start_config) / self.ComputeDistance(end_config, start_config) * max_extend + start_config
        
        numSamples = 100
        vals = numpy.array([numpy.linspace(i, j, numSamples) for i, j in zip(start_config, end_config)]).T

        for v in vals:
            if self.checkCollision(v):
                return None

        return end_config
        
    def ShortenPath(self, path, timeout=1.0):
        #  Implemented a function which performs path shortening
        #  on the given path.  Terminate the shortening after the 
        #  given timout (in seconds).

        path = numpy.array(path)

        start_time = time.time()
        while(time.time() - start_time < timeout and len(path) != 2):

            # sample point A
            a = randint(1, len(path)-2)
            u = uniform(0,1)   
            pointA = (1-u) * path[a-1] + u * path[a]

            # sample point B
            b = a
            while (b == a): # make sure we don't get the same point
                b = randint(1, len(path)-2)   
            u = uniform(0,1) # where to sample interpolated point
            pointB = (1-u) * path[b-1] + u * path[b]

            # self.clearPlot() # UNCOMMENT TO VISUALIZE
            # self.plotPath(path)
            # self.PlotPoint(pointA)
            # self.PlotPoint(pointB)
            # self.PlotEdge(pointA,pointB)

            # is there a collision free LOS? 
            if self.clearLineOfSight(pointA, pointB):

                if a < b: # trim from a to b
                    path = numpy.vstack((path[0:a], pointA, pointB, path[b:]))
                elif a > b:
                    path = numpy.vstack((path[0:b], pointB, pointA, path[a:]))
                else: 
                    print "there's a problem here!"
            
        # self.clearPlot() # UNCOMMENT TO VISUALIZE
        # self.plotPath(path)

        return path

    def clearLineOfSight(self, start_config, end_config, numSamples = 1000):
        vals = numpy.array([numpy.linspace(i, j, numSamples) for i, j in zip(start_config, end_config)]).T

        for v in vals:
            if self.checkCollision(v):
                return False

        return True

    def PlotEdge(self, sconfig, econfig):
        # placeholder
        pass

    def plotPath(self, arga, argb):
        #dummy function
        pass

    def getPathLength(self, path):
        
        dist = 0
        for i in range(0,len(path)-1):
            dist += self.ComputeDistance(path[i], path[i+1])
        return dist
