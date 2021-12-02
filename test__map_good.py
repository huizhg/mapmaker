# ------------------------------Done. Dont modify it!  

from robot import Robot
from math import pi
from show_map import ShowMap
import numpy as np
from sensor_model import Sensor_model
from map import Map
from bayesian import Bayesian
def main():
    cell_size = 1
    baye_map = Map(-70,-70,70,70,cell_size)
    # print(len(baye_map.grid))
    # print(baye_map.grid)
    # print("\n")
    bayesian = Bayesian(baye_map.grid,cell_size)
    # print(baye_map.n_row,baye_map.n_col,baye_map.scale)
    # print(len(bayesian.baye_map))
    # print(bayesian.baye_map)
    # print("\n")
    map = ShowMap(baye_map.n_row,baye_map.n_col,True)
    sensor_model = Sensor_model(cell_size)
    host = "http://127.0.0.1:50000"

    # make a robot to move around
    robot = Robot(host)

    # move the robot
    robot.setMotion(0, 0)

    while(1):
        curr_pos = robot.getPosition()
        print(curr_pos)
        robot_coord = pos_to_grid(curr_pos['X'], curr_pos['Y'], -70, 70, cell_size)
        robot_heading = robot.getHeading()
        laser_angles = robot.getLaserAngles()

        laser_scan = robot.getLaser()
        laser_distance = laser_scan['Echoes']
        # k=0
        # print(robot_coord)
        # print(robot_heading)
        # print(laser_angles)

        # print(laser_distance)
        points = sensor_model.get_detected_points_coord(robot_coord,robot_heading,laser_angles,laser_distance)
        lines = sensor_model.get_bresenham_lines(robot_coord,points)
        # print(lines[270])
        # print(lines[270][0][0],lines[270][0][1])
        for i in range(len(lines)):
            bayesian.baye_map_maker(lines[i],0,robot_coord[0],robot_coord[1])
            

        map.updateMap(bayesian.baye_map,1.2,robot_coord[0],robot_coord[1])
        
        robot.setMotion(2,0)
        # k=k+1
        # print(k)
        # if k >5:
        #     k=-1
        # elif k<0: 
        #     break
        # else:
        #     robot.setMotion(1,0)






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
    col = (x - xmin) / cellsize
    row = (ymax - y) / cellsize
    return (row, col)


if __name__ == '__main__':
    main()
