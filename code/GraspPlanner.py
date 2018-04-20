import logging, openravepy
from DiscreteEnvironment import DiscreteEnvironment
import numpy as np
np.random.seed(0)
import time
import math

class GraspPlanner(object):

    def __init__(self, robot, base_planner, arm_planner):
        self.robot = robot
        self.base_planner = base_planner
        self.arm_planner = arm_planner

        # Inverse Reachability Model
        self.irmodel = openravepy.databases.inversereachability.InverseReachabilityModel(self.robot)
        
        starttime = time.time()
        if not self.irmodel.load():
            print "inversereachability model failed to load, let's generate..."
            self.irmodel.autogenerate()
            self.irmodel.load()
        print 'time to load inverse-reachability model: %fs'%(time.time()-starttime)

            
    def GetBasePoseForObjectGrasp(self, obj):

        ###################################################################
        #  Here you will fill in the function to compute
        #  a base pose and associated grasp config for the 
        #  grasping the bottle
        ###################################################################

        base_pose = None
        grasp_config = None

        # Load grasp database
        gmodel = openravepy.databases.grasping.GraspingModel(self.robot, obj)
        if not gmodel.load():
            gmodel.autogenerate()
        
        grasps = gmodel.grasps
        grasp_indices = gmodel.graspindices

        # Ordering grasps
        if False:
            for grasp in grasps:
                grasp[grasp_indices.get('performance')] = self.eval_grasp(gmodel, grasp)
            order = np.argsort(grasps[:, grasp_indices.get('performance')[0]])
            order = order[::-1]
            grasp_config = grasps[order[0]] # select best grasp
        else:
            grasp_config = grasps[0] # skip odering to make it faster

        # Let's work with the our newly generated grasp 
        if not grasp_config is None:

            print "Here's our 'optimal' grasp"
            gmodel.showgrasp(grasp_config) 

            # Get Grasp Transform
            Tgrasp = gmodel.getGlobalGraspTransform(grasp_config, collisionfree = True)

            # get gaussian kernels for pose sampling around grasp tf
            densityfn, samplerfn, bounds = self.irmodel.computeBaseDistribution(Tgrasp)
            poses, jointstate = samplerfn(1000) # we can tune # samples

            # Store Initial Poses
            initial_config = self.robot.GetActiveDOFValues()
            initial_pose = self.robot.GetTransform()

            # Iterate through new poses to find a valid configuration
            for pose in poses:

                # Set to discrete pose 
                node = self.base_planner.planning_env.discrete_env.ConfigurationToNodeId(self.convertPose(pose))
                discrete_pose = self.base_planner.planning_env.discrete_env.NodeIdToConfiguration(node)
                
                # convert back again and check for collision
                base_pose = self.convertPoseBack(discrete_pose)
                self.robot.SetTransform(base_pose)
                obstacles = self.robot.GetEnv().GetBodies()
                if self.robot.GetEnv().CheckCollision(self.robot, obstacles[1]) == False:

                    # Find grasp config
                    self.robot.SetDOFValues(*jointstate)
                    grasp_config = self.robot.GetActiveManipulator().FindIKSolution(Tgrasp,
                        filteroptions=openravepy.IkFilterOptions.CheckEnvCollisions.IgnoreEndEffectorCollisions)

                    if not grasp_config is None: # check validity

                        ### PEEK AT DESIRED POSE ###
                        self.robot.SetActiveDOFValues(grasp_config)
                        self.robot.SetTransform(base_pose)
                        raw_input("Collision-free config found (enter)...")

                        self.robot.SetTransform(initial_pose)
                        self.robot.SetActiveDOFValues(initial_config)

                        return self.convertBasePose(base_pose), grasp_config


    def basePoseDistance(self, initial_pose, final_pose):
        return math.sqrt( (initial_pose[0][3] - final_pose[0][3])**2 + (initial_pose[1][3] - final_pose[1][3])**2 )


    def convertPose(self, H):
        angle = openravepy.axisAngleFromQuat(H)
        pose = [H[4], H[5], angle[2]]
        return pose


    def convertPoseBack(self, pose):
        H = np.identity(4)
        H[0][3] = pose[0]
        H[1][3] = pose[1]
        c, s = np.cos(pose[2]), np.sin(pose[2])
        R = np.matrix([[c, -s], [s, c]])
        H[0:2,0:2] = R
        return H


    def convertBasePose(self, H):
        x,y,z = self.mat2euler(H)
        pose = [H[0][3], H[1][3], z]
        return pose


    # https://afni.nimh.nih.gov/pub/dist/src/pkundu/meica.libs/nibabel/eulerangles.py
    def mat2euler(self, M, cy_thresh=None):
        M = M[0:3,0:3]
        M = np.asarray(M)
        if cy_thresh is None:
            try:
                cy_thresh = np.finfo(M.dtype).eps * 4
            except ValueError:
                cy_thresh = _FLOAT_EPS_4
        r11, r12, r13, r21, r22, r23, r31, r32, r33 = M.flat
        # cy: sqrt((cos(y)*cos(z))**2 + (cos(x)*cos(y))**2)
        cy = math.sqrt(r33*r33 + r23*r23)
        if cy > cy_thresh: # cos(y) not close to zero, standard form
            z = math.atan2(-r12,  r11) # atan2(cos(y)*sin(z), cos(y)*cos(z))
            y = math.atan2(r13,  cy) # atan2(sin(y), cy)
            x = math.atan2(-r23, r33) # atan2(cos(y)*sin(x), cos(x)*cos(y))
        else: # cos(y) (close to) zero, so x -> 0.0 (see above)
            # so r21 -> sin(z), r22 -> cos(z) and
            z = math.atan2(r21,  r22)
            y = math.atan2(r13,  cy) # atan2(sin(y), cy)
            x = 0.0
        return x, y, z


    def PlanToGrasp(self, obj):

        # Next select a pose for the base and an associated ik for the arm
        base_pose, grasp_config = self.GetBasePoseForObjectGrasp(obj)

        if base_pose is None or grasp_config is None:
            print 'Failed to find solution'
            exit()

        # Now plan to the base pose
        start_pose = np.array(self.base_planner.planning_env.herb.GetCurrentConfiguration())
        base_plan = self.base_planner.Plan(start_pose, base_pose)
        base_traj = self.base_planner.planning_env.herb.ConvertPlanToTrajectory(base_plan)

        print 'Executing base trajectory'
        self.base_planner.planning_env.herb.ExecuteTrajectory(base_traj)

        # Now plan the arm to the grasp configuration
        start_config = np.array(self.arm_planner.planning_env.herb.GetCurrentConfiguration())
        arm_plan = self.arm_planner.Plan(start_config, grasp_config)
        arm_traj = self.arm_planner.planning_env.herb.ConvertPlanToTrajectory(arm_plan)

        print 'Executing arm trajectory'
        self.arm_planner.planning_env.herb.ExecuteTrajectory(arm_traj)

        # Grasp the bottle
        task_manipulation = openravepy.interfaces.TaskManipulation(self.robot)
        task_manipulation.CloseFingers()


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


