# import logging, openravepy
import pdb
# import numpy as np
# np.random.seed(0)

import logging, openravepy
import os
import copy
import time
import math
import numpy as np
np.random.seed(0)
import scipy
from numpy import linalg
import time
from DiscreteEnvironment import DiscreteEnvironment
import IPython
import random

class GraspPlanner(object):

    def __init__(self, robot, base_planner, arm_planner):
        self.robot = robot
        self.base_planner = base_planner
        self.arm_planner = arm_planner
        # ADDED
        self.env = self.robot.GetEnv()
        self.manip = self.robot.GetActiveManipulator()

            
    def GetBasePoseForObjectGrasp(self, obj):

        print "\nGetBasePoseForObjectGrasp()...\n"

        # Load grasp database
        gmodel = openravepy.databases.grasping.GraspingModel(self.robot, obj)
        if not gmodel.load():
            gmodel.autogenerate()

        base_pose = None
        grasp_config = None

        ###################################################################
        #  Here you will fill in the function to compute
        #  a base pose and associated grasp config for the 
        #  grasping the bottle
        ###################################################################
        
        grasps = gmodel.grasps
        grasp_indices = gmodel.graspindices

        # for grasp in grasps:
        #     grasp[grasp_indices.get('performance')] = eval_grasp(gmodel, grasp)
        # order = np.argsort(grasps[:, grasp_indices.get('performance')[0]])
        # order = order[::-1]
        # grasp_config = grasps[order[0]] # select best grasp

        grasp_config = grasps[0] # select best grasp

        # gmodel.showgrasp(grasp_config)

        # Get Grasp Transform
        Tgrasp = gmodel.getGlobalGraspTransform(grasp_config, collisionfree = True)

        # Inverse Reachability Model
        irmodel = openravepy.databases.inversereachability.InverseReachabilityModel(self.robot)
        
        starttime = time.time()
        if not irmodel.load():
            print "inversereachability model failed to load, let's generate..."
            irmodel.autogenerate()
            irmodel.load()
        print 'time to load inverse-reachability model: %fs'%(time.time()-starttime)

        densityfn, samplerfn, bounds = irmodel.computeBaseDistribution(Tgrasp)

        # N = 1
        # # initialize sampling parameters
        # goals = []
        # numfailures = 0
        # starttime = time.time()
        # timeout = 1000
        # with self.robot:
        #     while len(goals) < N:
        #         if time.time()-starttime > timeout:
        #             break

        #         poses,jointstate = samplerfn(N-len(goals))

        #         for pose in poses:
        #             self.robot.SetTransform(pose)
        #             self.robot.SetDOFValues(*jointstate)
        #             # validate that base is not in collision
        #             if not self.manip.CheckIndependentCollision(CollisionReport()):
        #                 q = self.manip.FindIKSolution(Tgrasp,filteroptions=IkFilterOptions.CheckEnvCollisions)
        #                 if q is not None:
        #                     values = self.robot.GetDOFValues()
        #                     values[self.manip.GetArmIndices()] = q
        #                     goals.append((Tgrasp,pose,values))
        #                 elif self.manip.FindIKSolution(Tgrasp,0) is None:
        #                     numfailures += 1

        # pdb.set_trace()

        # initialize sampling parameters
        poses, jointstate = samplerfn(100)
        self.manip = self.robot.GetActiveManipulator()
        start_pose = self.robot.GetTransform()

        # init_config = self.base_planner.planning_env.herb.GetCurrentConfiguration()

        for pose in poses:

            self.robot.SetTransform(pose)
            self.robot.SetDOFValues(*jointstate)

            angle = openravepy.axisAngleFromQuat(pose)
            base_pose = copy.deepcopy([pose[4], pose[5], angle[2]])

            
            node = self.base_planner.planning_env.discrete_env.ConfigurationToNodeId(base_pose)
            discrete_pose = self.base_planner.planning_env.discrete_env.NodeIdToConfiguration(node)
            self.base_planner.planning_env.herb.SetCurrentConfiguration(discrete_pose)


            obs = self.robot.GetEnv().GetBodies()
            table = obs[1]

            grasp_config = self.manip.FindIKSolution(Tgrasp,
                filteroptions=openravepy.IkFilterOptions.CheckEnvCollisions.IgnoreEndEffectorCollisions)

            if not grasp_config is None and self.robot.GetEnv().CheckCollision(self.robot, table) == False:

                print "grasp_config", grasp_config
                print "base_pose", base_pose

                # self.base_planner.planning_env.herb.SetCurrentConfiguration(init_config)

                return  base_pose, grasp_config # discrete_pose 



    def PlanToGrasp(self, obj):

        # Next select a pose for the base and an associated ik for the arm
        base_pose, grasp_config = self.GetBasePoseForObjectGrasp(obj)

        if base_pose is None or grasp_config is None:
            print 'Failed to find solution'
            exit()

        # Now plan to the base pose
        start_pose = numpy.array(self.base_planner.planning_env.herb.GetCurrentConfiguration())
        base_plan = self.base_planner.Plan(start_pose, base_pose)
        base_traj = self.base_planner.planning_env.herb.ConvertPlanToTrajectory(base_plan)

        print 'Executing base trajectory'
        self.base_planner.planning_env.herb.ExecuteTrajectory(base_traj)

        # Now plan the arm to the grasp configuration
        start_config = numpy.array(self.arm_planner.planning_env.herb.GetCurrentConfiguration())
        arm_plan = self.arm_planner.Plan(start_config, grasp_config)
        arm_traj = self.arm_planner.planning_env.herb.ConvertPlanToTrajectory(arm_plan)

        print 'Executing arm trajectory'
        self.arm_planner.planning_env.herb.ExecuteTrajectory(arm_traj)

        # Grasp the bottle
        task_manipulation = openravepy.interfaces.TaskManipulation(self.robot)
        task_manipultion.CloseFingers()


    def eval_grasp(self, gmodel, grasp):
       
       with self.robot:
           #contacts is a 2d array, where contacts[i,0-2] are the positions of contact i and contacts[i,3-5] is the direction
          try:
            contacts,finalconfig,mindist,volume = gmodel.testGrasp(grasp=grasp,translate=True,forceclosure=False)
            
            obj_position = gmodel.target.GetTransform()[0:3,3]
            # for each contact
            G = np.zeros([6, 0]) #the wrench matrix
            for c in contacts:
              pos = c[0:3] - obj_position
              dir = -c[3:] #this is already a unit vector
              g = np.expand_dims(np.concatenate((dir, np.cross(pos,dir))), axis = 1)
              G = np.concatenate((G, g), axis = 1)
              
            w, v =  np.linalg.eig(np.dot(G, G.T))
            eigs = np.sqrt(w + 0.000001)
            Q = np.min(eigs)
            return Q #change this

          except openravepy.planning_error,e:
            #you get here if there is a failure in planning
            #example: if the hand is already intersecting the object at the initial position/orientation
            return  0.00 # TODO you may want to change this


