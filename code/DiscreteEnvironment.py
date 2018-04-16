import numpy as np

class DiscreteEnvironment(object):

    def __init__(self, resolution, lower_limits, upper_limits):

        # Store the resolution
        self.resolution = resolution

        # Store the bounds
        self.lower_limits = lower_limits
        self.upper_limits = upper_limits

        # Calculate the dimension
        self.dimension = len(self.lower_limits)

        # Figure out the number of grid cells that are in each dimension
        self.num_cells = self.dimension*[0]
        for idx in range(self.dimension):
            self.num_cells[idx] = np.ceil((upper_limits[idx] - lower_limits[idx])/resolution[idx])

    def ConfigurationToNodeId(self, config):
        grid = self.ConfigurationToGridCoord(config)
        return self.GridCoordToNodeId(grid)

    def NodeIdToConfiguration(self, nid):
        grid = self.NodeIdToGridCoord(nid)
        return self.GridCoordToConfiguration(grid)

    def ConfigurationToGridCoord(self, config):
        config = np.clip(config, self.lower_limits, self.upper_limits)
        return ((np.array(config) - self.lower_limits) // self.resolution).astype(np.uint)

    def GridCoordToConfiguration(self, coord):
        return (np.array(coord) + 0.5) * self.resolution + self.lower_limits

    def GridCoordToNodeId(self,coord):
        return np.sum(np.cumprod([1] + self.num_cells[:-1]) * coord)

    def NodeIdToGridCoord(self, node_id):
        return node_id % np.cumprod(self.num_cells) // np.cumprod([1] + self.num_cells[:-1])

        
        
        
