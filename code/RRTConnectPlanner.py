import numpy, operator
from RRTPlanner import RRTTree

class RRTConnectPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        

    def Plan(self, start_config, goal_config, epsilon = 0.3):
        
        ftree = RRTTree(self.planning_env, start_config)
        rtree = RRTTree(self.planning_env, goal_config)
        plan = []

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        goalDistThreshold = epsilon
        goalDist = self.planning_env.ComputeDistance(start_config, goal_config)

        sgoal = start_config.copy()
        egoal = goal_config.copy()

        while goalDist > goalDistThreshold:

            fnew_config = self.planning_env.GenerateRandomConfiguration_GoalBias(egoal)
            fclosest_id, fclosest_config = ftree.GetNearestVertex(fnew_config)
            fnear_config = (fnew_config - fclosest_config) * min(1, (epsilon / self.planning_env.ComputeDistance(fclosest_config, fnew_config))) + fclosest_config
            fnew_config = self.planning_env.Extend(fclosest_config, fnear_config, epsilon)

            if fnew_config is not None:
                fnew_id = ftree.AddVertex(fnew_config)
                ftree.AddEdge(fclosest_id, fnew_id)
                self.planning_env.PlotEdge(fclosest_config, fnew_config) # PLOT

                while True:
                    rclosest_id, rclosest_config = rtree.GetNearestVertex(fnew_config)
                    rnear_config = (fnew_config - rclosest_config) * min(1, (epsilon / self.planning_env.ComputeDistance(rclosest_config,
                                                                                                      fnew_config))) + rclosest_config
                    rnew_config = self.planning_env.Extend(rclosest_config, rnear_config, epsilon)
                    if rnew_config is not None:
                        rnew_id = rtree.AddVertex(rnew_config)
                        rtree.AddEdge(rclosest_id, rnew_id)
                        self.planning_env.PlotEdge(rclosest_config, rnew_config) # PLOT
                    else:
                        ftree, rtree = rtree, ftree
                        sgoal, egoal = egoal, sgoal
                        break

                    goalDist = self.planning_env.ComputeDistance(fnew_config, rnew_config)

                    if goalDist <= goalDistThreshold:
                        break

        fparent_id = fnew_id
        fparent_config = fnew_config

        fplan = []
        while fparent_id is not None:
            fplan.append(fparent_config)
            fchild_id = fparent_id
            fchild_config = fparent_config
            fparent_id, fparent_config = ftree.getParent(fchild_id)

        rparent_id = rnew_id
        rparent_config = rnew_config

        rplan = []
        while rparent_id is not None:
            rplan.append(rparent_config)
            rchild_id = rparent_id
            rchild_config = rparent_config
            rparent_id, rparent_config = rtree.getParent(rchild_id)

        if (fchild_config == start_config).all():
            plan = fplan[::-1] + rplan
        else:
            plan = rplan[::-1] + fplan

        return plan
