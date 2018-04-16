import numpy as np, openravepy
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment

class Control(object):
    def __init__(self, omega_left, omega_right, duration):
        self.ul = omega_left
        self.ur = omega_right
        self.dt = duration

class Action(object):
    def __init__(self, control, footprint):
        self.control = control
        self.footprint = footprint

class SimpleEnvironment(object):
    
    def __init__(self, herb, resolution):
        self.herb = herb
        self.robot = herb.robot
        self.boundary_limits = [[-5., -5., -np.pi], [5., 5., np.pi]]
        lower_limits, upper_limits = self.boundary_limits
        self.discrete_env = DiscreteEnvironment(resolution, lower_limits, upper_limits)
        # print(self.discrete_env.num_cells)
        self.resolution = resolution
        # self.ConstructActions()

    def GenerateFootprintFromControl(self, start_config, control, stepsize=0.01):

        # Extract the elements of the control
        ul = control.ul
        ur = control.ur
        dt = control.dt

        # Initialize the footprint
        config = start_config.copy()
        # footprint = [np.array([0., 0., config[2]])]
        footprint = [config.copy()]

        timecount = 0.0
        while timecount < dt:
            # Generate the velocities based on the forward model
            xdot = 0.5 * self.herb.wheel_radius * (ul + ur) * np.cos(config[2])
            ydot = 0.5 * self.herb.wheel_radius * (ul + ur) * np.sin(config[2])
            tdot = self.herb.wheel_radius * (ul - ur) / self.herb.wheel_distance
                
            # Feed forward the velocities
            if timecount + stepsize > dt:
                stepsize = dt - timecount
            config = config + stepsize*np.array([xdot, ydot, tdot])
            if config[2] > np.pi:
                config[2] -= 2.*np.pi
            if config[2] < -np.pi:
                config[2] += 2.*np.pi

            # footprint_config = config.copy()
            # footprint_config[:2] -= start_config[:2]
            footprint.append(config.copy())

            timecount += stepsize
            
        # Add one more config that snaps the last point in the footprint to the center of the cell
        nid = self.discrete_env.ConfigurationToNodeId(config)
        snapped_config = self.discrete_env.NodeIdToConfiguration(nid)
        # snapped_config[:2] -= start_config[:2]
        footprint.append(snapped_config)

        return footprint

    def PlotActionFootprints(self, actions):

        fig = pl.figure()
        lower_limits, upper_limits = self.boundary_limits
        pl.xlim([lower_limits[0], upper_limits[0]])
        pl.ylim([lower_limits[1], upper_limits[1]])
        
        for action in actions:
            xpoints = [config[0] for config in action.footprint]
            ypoints = [config[1] for config in action.footprint]
            pl.plot(xpoints, ypoints, 'k')
                     
        pl.ion()
        pl.show()

        

    def ConstructActions(self, coord):

        actions = []
        turn_speed = 0.1
        move_speed = 1
        time_step = 1
        control_fw = Control(move_speed, move_speed, time_step)
        control_bw = Control(-move_speed, -move_speed, time_step)
        control_l = Control(-turn_speed, turn_speed, time_step)
        control_r = Control(turn_speed, -turn_speed, time_step)
        control_l2 = Control(-turn_speed * 2, turn_speed* 2, time_step)
        control_r2 = Control(turn_speed* 2, -turn_speed* 2, time_step)
        control_l3 = Control(-turn_speed* 3, turn_speed* 3, time_step)
        control_r3 = Control(turn_speed* 3, -turn_speed* 3, time_step)
        control_l4 = Control(-turn_speed* 4, turn_speed* 4, time_step)
        control_r4 = Control(turn_speed* 4, -turn_speed* 4, time_step)

        config = self.discrete_env.GridCoordToConfiguration(coord)
        # coord = self.discrete_env.NodeIdToGridCoord(i)


        actions.append(Action(control_fw, self.GenerateFootprintFromControl(config, control_fw)))
        actions.append(Action(control_bw, self.GenerateFootprintFromControl(config, control_bw)))
        # actions.append(Action(control_l, self.GenerateFootprintFromControl(config, control_l)))
        # actions.append(Action(control_r, self.GenerateFootprintFromControl(config, control_r)))
        # actions.append(Action(control_l2, self.GenerateFootprintFromControl(config, control_l2)))
        # actions.append(Action(control_r2, self.GenerateFootprintFromControl(config, control_r2)))
        # actions.append(Action(control_l3, self.GenerateFootprintFromControl(config, control_l3)))
        # actions.append(Action(control_r3, self.GenerateFootprintFromControl(config, control_r3)))
        actions.append(Action(control_l4, self.GenerateFootprintFromControl(config, control_l4)))
        actions.append(Action(control_r4, self.GenerateFootprintFromControl(config, control_r4)))

        return actions


    def GetSuccessors(self, coord):

        actions = self.ConstructActions(coord)
        return [(actions[i], self.discrete_env.ConfigurationToGridCoord(actions[i].footprint[-1])) for i in range(len(actions))]

    def ComputeDistance(self, start_coord, end_coord,):
        return np.linalg.norm(self.discrete_env.GridCoordToConfiguration(start_coord) - self.discrete_env.GridCoordToConfiguration(end_coord))    

    def ComputeHeuristicCost(self, start_coord, end_coord):
        #Use distance as heuristic?
        return np.linalg.norm(self.discrete_env.GridCoordToConfiguration(start_coord) - self.discrete_env.GridCoordToConfiguration(end_coord))    


    def checkCollision(self, coord):
        pose = self.discrete_env.GridCoordToConfiguration(coord)
        self.herb.SetCurrentConfiguration(pose)
        return False
        robot_saver = self.robot.CreateRobotStateSaver(
              self.robot.SaveParameters.ActiveDOF
            | self.robot.SaveParameters.ActiveManipulator
            | self.robot.SaveParameters.LinkTransformation)
        config = self.discrete_env.GridCoordToConfiguration(coord)
        env = self.robot.GetEnv()
        lower_limits, upper_limits = self.boundary_limits

        if (lower_limits > config).any() or (upper_limits < config).any():
            return True

        with robot_saver, env:

            return self.robot.GetEnv().CheckCollision(self.robot.GetEnv().GetBodies()[1])


        # pose = self.discrete_env.GridCoordToConfiguration(coord)
        # # # print(pose)
        #
        # # T = np.array([ [ 1, 0,  0, pose[0,0]],
        # #                   [ 0, 1,  0, pose[0,1]],
        # #                   [ 0, 0,  1, 0],
        # #                   [ 0, 0,  0, 1]])
        # self.herb.SetCurrentConfiguration(pose)
        # return self.robot.GetEnv().CheckCollision(self.robot.GetEnv().GetBodies()[1])
