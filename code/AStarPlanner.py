import numpy as np
from collections import deque
from itertools import count
from heapq import *
import os
np.set_printoptions(threshold=np.inf)

class AStarPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()


    def Plan(self, start_config, goal_config):

        start_coord = self.planning_env.discrete_env.ConfigurationToGridCoord(start_config)
        goal_coord = self.planning_env.discrete_env.ConfigurationToGridCoord(goal_config)
        neighbors = self.planning_env.GetSuccessors(start_coord)

        if self.visualize:
            self.planning_env.InitializePlot(goal_config)

        h = [] #Use a heap
        parents = {}
        parents[tuple(start_coord)] = None
        curr_cost = {}
        tb = count()

        for n in neighbors:
            print(n[1])
            heappush(h, (self.planning_env.ComputeDistance(n[1], start_coord)
                         + self.planning_env.ComputeHeuristicCost(n[1], goal_coord), 
                        self.planning_env.ComputeDistance(n[1], start_coord),
                        next(tb), n)) #Total cost, current cost, tiebreaker num, action object
            parents[tuple(n[1])] = (start_coord, n[0])

        # print("main loop")
        while True:
            #pop min element from heap
            node = heappop(h)

            #Add neighbors
            neighbors = self.planning_env.GetSuccessors(node[3][1])

            for n in neighbors:
                # print("neighors")
                # print(len(parents))
                #OOB
                if (n[1] < 0).any() or (n[1] >= self.planning_env.discrete_env.num_cells).any():
                    # print("oob")
                    continue

                #Explored
                if tuple(n[1]) in parents:
                    # print("explored")
                    continue
                    # if parents[tuple(n)][1] <= node[1] + self.planning_env.ComputeDistance(node[3], n):
                    #     continue
                    # else:
                    #     for H in h:
                    #         if (H[3] == n).all():
                    #             h.remove(H)
                    #             break
                    #     heapify(h)


                #Collision
                if self.planning_env.checkCollision(n[1]):
                    # print("collision")
                    continue

                #visualize
                if self.visualize:
                    self.planning_env.PlotEdge(np.squeeze(self.planning_env.discrete_env.GridCoordToConfiguration(node[3][1])).copy(), 
                        np.squeeze(self.planning_env.discrete_env.GridCoordToConfiguration(n)).copy())

                #Reached the end
                # print(goal_coord)
                # print(n[1])
                # os.system('clear')
                if (n[1] == goal_coord).all():
                    parents[tuple(n[1])] = (node[3][1], n[0])
                    return self.createPath(parents, n)
                #Add parents
                heappush(h, (node[1] + self.planning_env.ComputeDistance(node[3][1], n[1]) 
                    + self.planning_env.ComputeHeuristicCost(n[1], goal_coord), 
                    node[1] + self.planning_env.ComputeDistance(node[3][1], n[1]), next(tb), n))
                parents[tuple(n[1])] = (node[3][1], n[0])
        print("Should never reach here")

    def createPath(self, parents, n):
        print("number of explored nodes: " + str(len(parents)))

        path = []
        coord = n[1]
        while True:
            if parents[tuple(coord)] is None:
                break
            path.append(parents[tuple(coord)][1])
            coord = parents[tuple(coord)][0]

        return path[::-1]



        # path.append(np.expand_dims(goal, axis = 0))

        # while True:
        #     path.append(self.planning_env.discrete_env.GridCoordToConfiguration(coord))
        #     coord = parents[tuple(coord)][0]
        #     if coord is None:
        #         break

        # path.append(np.expand_dims(start, axis = 0))
        # return np.concatenate(path[::-1], axis = 0)
