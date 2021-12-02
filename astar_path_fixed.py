from heapq import heappop, heappush


class Node(object):
	"""docstring for Node"""
	def __init__(self, parent, position):
		self.parent = parent
		self.position = position
		# f = g + h 
		self.f = 0
		self.g = 0
		self.h = 0


def astar_path(map, start, end):
	"""
	A astar path planning function:
	Args
		map: Map object
		start: tuple, start square in grid map.
		end: tuple, end square in grid map.
	Return
		Path: List. From start to end.
	"""
	# Inspired by https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2
	# Follow the Reference: https://www.gamedev.net/reference/articles/article2003.asp 
	open_list = []
	closed_list = []
	start_node = Node(None, start)
	end_node = Node(None, end)
	# Add the starting node to the open list
	open_list.append(start_node)
	
	while len(open_list)>0 :
		# print(len(open_list))
		# 5.a) Look for the lowest F cost square on the open list
		current_node = open_list[0]
		for item in open_list:
			# print("ind",ind)
			# print("item",item.position)
			if current_node.f > item.f:
				current_node = item
		# 5.b) switch it to the closed list
		open_list.remove(current_node)
		closed_list.append(current_node)
		# Exit of Loop
		if current_node.position == end_node.position: # if the current node is the goal, backtrack to get path
			# print("Finally")
			path = []
			current = current_node
			while current.parent is not None:
				path.append(current.position)
				current = current.parent
			return path[::-1]
		# Check all neighbors
		open_neighbors = map.get_open_neighbors_ex(current_node.position)
		# print(len(open_neighbors))

		if not len(open_neighbors) == 0:
			# print("Has someting?")
			for open_neighbor in open_neighbors:
				open_neighbor_node = Node(current_node, open_neighbor)
				# If open neighbor is in the closed list, continue.
				if open_neighbor_node in closed_list:
					continue

				# Calculate node values
				open_neighbor_node.g = current_node.g + 1 
				open_neighbor_node.h = heuristic_distance(open_neighbor, end_node.position)
				open_neighbor_node.f = open_neighbor_node.g + open_neighbor_node.h
				# print("Items?",open_list.check())
				# break

				# If it is on the open list already, check the G cost. 
				# If this square has lower G cost, change the parent of the square to the current square and recalculate the G, F.
				if len([item for item in open_list if open_neighbor_node == item and open_neighbor_node.g > item.g]):
					continue
				# for item in open_list:
				# 	if open_neighbor_node == item and open_neighbor_node.g > item.g:
				# 		continue
				# If it is not on the open list, add it to the open list.
				open_list.append(open_neighbor_node)
				if (len(open_list)>5000):
					return []
				print(len(open_list))

def heuristic_distance(cell_1, cell_2):
	# Manhattan distance
    return ((cell_2[0] - cell_1[0]) ** 2 + (cell_2[1] - cell_1[1]) ** 2)
	
		
