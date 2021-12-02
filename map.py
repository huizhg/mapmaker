import numpy as np 

class Map(object):
	"""An occupancy grid map """
	OPEN_THRESHOLD = 0.3
	OCCUPIED_THRESHOLD = 0.7

	def __init__(self, x_min, y_min, x_max, y_max, cell_size):
		"""
		Arguments:
		x_min -- bottom left x coordinate of the map object
		y_min -- bottom left y coordinate of the map object
		x_max -- top right x coordinate of the map object
		y_max -- top rigth y coordinate of the map object
		cell_size --  the resolution of the grid, (how many meters per grid cell)

		"""
		self.x_min = x_min
		self.y_min = y_min
		self.x_max = x_max
		self.y_max = y_max
		self.cell_size = cell_size
		self.scale = 1 / cell_size
		self.n_col = int(abs(x_max - x_min)*self.scale) # number of columns of the grid map
		self.n_row = int(abs(y_max - y_min)*self.scale) # number of rows of the grid map
		self.grid = np.ones(shape = (self.n_row, self.n_col))*0.5 # initial value of the occupancy grid map, 0.7 represents unknown probability
		self.expanded_grid = self.grid

	def get_size(self):
		"""
		Return:
		A tuple, contains the size of the occupancy grid map
		"""
		return (self.n_row, self.n_col)

	def convert_to_grid_idx(self, x, y):
		"""
		Convert the real coordinate(x,y) to the grid map index (row, col)

		Arguments:
		(x, y) -- The real coordinates

		Retrun:
		(row, col) -- The row, and col index of the real position in the grid
		"""
		col = int(abs(x - self.x_min)*self.scale)
		row = int(abs(self.y_max - y)*self.scale)
		return row, col 

	def is_obstacle(self, cell):
		row = cell[0]
		col = cell[1]
		return self.grid[row][col] >= 0.7

	def is_obstacle_ex(self, cell):
		row = cell[0]
		col = cell[1]
		return self.expanded_grid[row][col] >= 0.7

	def is_free(self, cell):
		row = cell[0]
		col = cell[1]
		return self.grid[row][col] <= 0.3

	def is_free_ex(self, cell):
		row = cell[0]
		col = cell[1]
		return self.expanded_grid[row][col] <= 0.3

	def is_unknown(self, cell):
		row = cell[0]
		col = cell[1]		
		return self.grid[row][col] > 0.3 and self.grid[row][col] < 0.7

	def is_unknown_ex(self, cell):
		row = cell[0]
		col = cell[1]		
		return self.expanded_grid[row][col] > 0.3 and self.expanded_grid[row][col] < 0.7		

	def is_within_grid(self, cell):
		row = cell[0]
		col = cell[1]
		return  0 <= row < self.n_row and 0 <= col < self.n_col

	def get_occupancy(self, cell):
		row = cell[0]
		col = cell[1]
		return self.grid[row][col]

	def get_occupancy_ex(self, cell):
		row = cell[0]
		col = cell[1]
		return self.expanded_grid[row][col]

	def set_occupancy(self, cell, p):
		row = cell[0]
		col = cell[1]
		self.grid[row][col] = p

	def set_occupancy_expanded_grid(self, cell, p):
		row = cell[0]
		col = cell[1]
		self.expanded_grid[row][col] = p


	def get_neighbors(self, cell):
		"""
		Get 8 neighbors of of (row,col) if they exist
		"""
		row = cell[0]
		col = cell[1]
		neighbors = []
		directions_list = [(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]
		if not self.is_within_grid(cell):
			return []

		for direction in directions_list:
			neighbor_cell = (row + direction[0], col + direction[1])
			if self.is_within_grid(neighbor_cell):
				neighbors.append(neighbor_cell)
			else:
				continue
		return neighbors

	
	def get_open_neighbors(self,cell):
		neighbors = self.get_neighbors(cell)
		open_neighbors = []
		for neighbor in neighbors:
			if self.is_free(neighbor):
				open_neighbors.append(neighbor)
		return open_neighbors

	def get_open_neighbors_ex(self,cell):
		neighbors = self.get_neighbors(cell)
		open_neighbors = []
		for neighbor in neighbors:
			if self.is_free_ex(neighbor):
				open_neighbors.append(neighbor)
		return open_neighbors	
	

	def has_open_neighbor(self, cell):
		neighbors = self.get_neighbors(cell)
		for neighbor in neighbors:
			if self.is_free(neighbor):
				return True
		return False

	def has_open_neighbor_ex(self, cell):
		neighbors = self.get_neighbors(cell)
		for neighbor in neighbors:
			if self.is_free_ex(neighbor):
				return True
		return False	

	
	def is_frontier_point(self, cell):
		"""
		A frontier point is an unknown point which has at least one open neighbor
		"""
		p = self.get_occupancy(cell)
		return p>0.1 and p<0.7 and self.has_open_neighbor(cell)

	def is_frontier_point_ex(self, cell):
		"""
		A frontier point is an unknown point which has at least one open neighbor
		"""
		p = self.get_occupancy_ex(cell)
		return p>0.1 and p<0.7 and self.has_open_neighbor_ex(cell)

	def expand_obstacles(self):
		"""
		Set all 8 neighbors of the occupied cell as occupied as well, save the new grid to self.expanded_grid
		"""
		self.expanded_grid = self.grid.copy()
		for row in range(self.n_row):
			for col in range(self.n_col):
				if self.is_obstacle((row,col)): # loop throuth the grid and find the occupied cells
					self.expanded_grid[row][col] = 1
					neighbors = self.get_neighbors((row,col))
					if len(neighbors) == 0:
						continue
					else:
						for neighbor in neighbors:
							self.set_occupancy_expanded_grid(neighbor,1)




