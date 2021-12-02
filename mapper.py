from robot import Robot
from math import floor
from show_map import ShowMap
import numpy as np
from sensor_model import Sensor_model
from map import Map
from bayesian import Bayesian
from goal_finder import Goal_finder
from astar_path_fixed import astar_path,Node
def main():
    """
        Mapper of project. It is usually named with test_xxx.py. 
    """
    # Prepare public parameter and object
    cell_size = 1
    baye_map = Map(-70,-70,70,70,cell_size)
    bayesian = Bayesian(baye_map.grid,cell_size)
    map = ShowMap(baye_map.n_row,baye_map.n_col,True)
    sensor_model = Sensor_model(cell_size)
    host = "http://127.0.0.1:50000"


    robot = Robot(host)
    robot.setMotion(0, 0)











    return None

def pos_to_grid(x, y, xmin, ymax, cellsize):
    """
    Converts an (x,y) positon to a (row,col) coordinate in the grid
    :param x: x-position
    :param y: y-position
    :param xmin: The minimum x-position in the grid
    :param ymax: The maximum y-position in the grid
    :param cellsize: the resolution of the grid
    :return: A tuple with (row,col)
    """
    col = floor((x - xmin) / cellsize) 
    row = floor((ymax - y) / cellsize) 
    return (row, col)

if __name__ == '__main__':
    main()