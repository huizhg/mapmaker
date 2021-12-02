# ------------------------------Done. Dont modify it!  
from robot import Robot
from math import pi
from show_map import ShowMap
import numpy as np
from sensor_model import Sensor_model
from map import Map
from bayesian import Bayesian
from goal_finder import Goal_finder
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
    k = 0
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
            

        map.updateMap(baye_map.grid,1.0,robot_coord[0],robot_coord[1])

            
        # print(k)
        if k >100:
            k=-1
        elif k<0:
            robot.setMotion(0,0)
            with open('your_file1.txt', 'w') as f:
                for item in bayesian.baye_map:
                    f.write("%s\n" % item)
            # with open('your_file2.txt', 'w') as f:
            #     for item in baye_map.grid:
            #         f.write("%s\n" % item)
            # for x in range(140):
            #     for y in range (140):
            #         if bayesian.baye_map[x,y]>0.9 and baye_map.grid[x,y]>0.9:
            #             print("ob1 = ",x,y,bayesian.baye_map[x,y]) 
            #             print("ob2 = ",x,y,baye_map.grid[x,y])
            #             print("ob3 = ",x,y,baye_map.get_occupancy((x,y)))           
            goalfinder = Goal_finder()
            # print(bayesian.baye_map)
            # print(baye_map.grid)
            # print("goal_parameter",robot_coord)
            print(goalfinder.find_goals(baye_map,robot_coord))
            break
        else:
            k=k+1
            robot.setMotion(-30,0)



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
