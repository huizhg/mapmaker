# Author Ola Ringdahl, ringdahl@cs.umu.se

from robot import *
from math import pi
from show_map import ShowMap
import numpy as np
from sensor_model import *
from carrot_controller import RCS_angle_calculation

def main():
    host = "http://127.0.0.1:50000"

    # make a robot to move around
    robot = Robot(host)

    # move the robot
    robot.setMotion(0, 0)


    cell_size = 1
    curr_pos = robot.getPosition()
    print(curr_pos)
    robot_coord = pos_to_grid(curr_pos['X'], curr_pos['Y'], -70, 70, cell_size)
    robot_heading = robot.getHeading()
    laser_angles = robot.getLaserAngles()

    laser_scan = robot.getLaser()
    laser_distance = laser_scan['Echoes']
    sensor_model = Sensor_model(cell_size)

    print(robot_coord)
    print("Heading",robot_heading)
    print(laser_distance)
    points = sensor_model.get_detected_points_coord(robot_coord,robot_heading,laser_angles,laser_distance)
    lines = sensor_model.get_bresenham_lines(robot_coord,points)

    for li in lines:
        for po in li:
            if po[0] >140:
                print("out of range: ", po)
            if po[1] > 140:
                print("out of range: ", po)

    # for i in range(0,len(points[0])):
    #     print(points[0][i],points[1][i])







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
