import numpy as np
from map import Map # there are differences between Map and map.
from show_map import ShowMap

"""
test map.py
"""
def main():
	grid_map = Map(0,0,5,5,0.5)
	print(grid_map.grid)
	n_row, n_col = grid_map.get_size()
	print("The grid has %d rows, %d cols "%(n_row, n_col))
	neighbors = grid_map.get_neighbors((3,5))
	if len(neighbors) > 0:
		print("cell(3,5) has %d neighbors" % len(neighbors))
		for neighbor in neighbors:
			print("(%d, %d) is a neighbor of (3,5)"%(neighbor[0], neighbor[1]))
	grid_map.set_occupancy((2,4),0.301)
	print(grid_map.is_frontier_point((2,5)))
	# grid_map.set_occupancy((2,5), 0.8)
	# grid_map.expand_obstacles()
	# show_map = ShowMap(n_row,n_col, showGUI = True)
	# grid_map.set_occupancy((4,7), 0.2)
	# print(grid_map.is_frontier_point((3,7)))
	# print(grid_map.expanded_grid)
	# show_map.updateMap(grid = grid_map.expanded_grid, maxValue = 255, robot_row = 9, robot_col = 2)
	# show_map.close()


if __name__ == '__main__':
	main()