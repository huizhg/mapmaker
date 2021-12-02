from collections import deque
from math import floor

class Goal_finder(object):
	"""Find the goal from bayesian map for controller"""

	
	def find_goals(self, map, robot_cell):
		"""
		Used to find frontiers given an occupancy grid map and the current position of the robot
		Implement the wavefront frontier detector (WFD) algorithm, algorithm is from 
		https://arxiv.org/ftp/arxiv/papers/1806/1806.03581.pdf
		"Frontier Based Exploration for Autonomous Robot" Anirudh et al. 
		"""
		# k1 = k2 = k3 = 0
		frontiers = []
		map_open_list = []
		map_close_list = []
		frontier_open_list =[]
		frontier_close_list =[]

		robot_cell = (int(robot_cell[0]),int(robot_cell[1]))
		queue_m = deque([])
		# enqueue pose
		queue_m.append(robot_cell)
		# mark pose as map open list
		map_open_list.append(robot_cell)

		while queue_m:
			
			p = queue_m.popleft()
			# print("point p:",p)
			if p in map_close_list:
				continue
			# -----Loop for f
			if map.is_frontier_point_ex(p):
				# print("find an edge point")
				queue_f = deque([])
				new_frontier = []
				queue_f.append(p)
				frontier_open_list.append(p)

				while queue_f:
					
					q = queue_f.popleft()

					if q in map_close_list and q in frontier_close_list:
						continue
					if map.is_frontier_point_ex(q):
						new_frontier.append(q)

						for w in map.get_neighbors(q):
							if w not in frontier_open_list and w not in frontier_close_list and w not in map_close_list:
								queue_f.append(w)
								frontier_open_list.append(w)
					
					frontier_close_list.append(q)
				
				frontiers.append(new_frontier)
				frontier_close_list.append(new_frontier)
			# -----End loop for f
			for v in map.get_open_neighbors_ex(p):
				if v not in map_open_list and v not in map_close_list and map.has_open_neighbor_ex(v):
					queue_m.append(v)
					map_open_list.append(v)

			map_close_list.append(p)


		goals = self.find_goals_from_frontiers(map, frontiers)
		return goals
		# return frontiers
	def find_goals_from_frontiers(self, map, frontiers):
		"""
        Get the list of frontiers and calculate their centroid

        Args
            frontiers: the frontiers found in the probability grid
        Return
            a list of tuples describing the centroid of each frontier
        """
		goals_list = []
		# op_goals_list = [] plan to merge close goals in one goal. 
		for frontier in frontiers:
			sum_x = 0
			sum_y = 0
			for x,y in frontier:
				sum_x = sum_x + x
				sum_y = sum_y + y
			length = len(frontier)
			goal = (floor(sum_x/length),floor(sum_y/length))
			if map.get_occupancy_ex(goal) < 0.2 and length > 4:
				goals_list.append(goal)
		


		return goals_list

