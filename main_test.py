from robot import Robot,time
from math import floor,sqrt
from show_map import ShowMap
import numpy as np
from sensor_model import Sensor_model
from map import Map
from bayesian import Bayesian
from goal_finder import Goal_finder
from astar_path_fixed import astar_path,Node
from carrot_controller import RCS_angle_calculation, obstacle_detector, pick_a_path
def main():
    """
        Mapper of project. It is usually named with test_xxx.py. 
    """
    # Prepare public parameter and object
    cell_size = 1
    baye_map = Map(-60,-60,60,60,cell_size)
    bayesian = Bayesian(baye_map.grid,cell_size)
    map = ShowMap(baye_map.n_row,baye_map.n_col,True)
    sensor_model = Sensor_model(cell_size)
    host = "http://127.0.0.1:50000"


    robot = Robot(host)
    robot.setMotion(0, 0)

    k = 0
    endflag = False
    # Stop at the start and check the surrounding state.
    while(1):
        curr_pos = robot.getPosition()
        robot_coord = pos_to_grid(curr_pos['X'], curr_pos['Y'], -60, 60, cell_size)
        robot_heading = robot.getHeading()
        laser_angles = robot.getLaserAngles()

        laser_scan = robot.getLaser()
        laser_distance = laser_scan['Echoes']

        points = sensor_model.get_detected_points_coord(robot_coord,robot_heading,laser_angles,laser_distance)
        lines = sensor_model.get_bresenham_lines(robot_coord,points)

        for i in range(len(lines)):
            bayesian.baye_map_maker(lines[i],0,robot_coord[0],robot_coord[1])

        baye_map.expand_obstacles()
        map.updateMap(baye_map.grid,1.0,robot_coord[0],robot_coord[1])

        goalfinder = Goal_finder()

        if k >50:
            k=-1
        elif k<0:         
            # goal = goalfinder.find_goals(baye_map,robot_coord)
            # if len(goal) > 0:
            #     print(robot_coord)
            #     print(goal[0])
            #     path = astar_path(baye_map,robot_coord,goal[0])
            #     print("path:",path)
            break
        else:
            k=k+1
            robot.setMotion(0,0.5)
    
       
    while(1):
        curr_pos = robot.getPosition()
        robot_coord = pos_to_grid(curr_pos['X'], curr_pos['Y'], -60, 60, cell_size)
        laser_scan = robot.getLaser()
        laser_distance = laser_scan['Echoes']
        # 1. Check the obstacle and prepare path for next step
        robot.setMotion(0,0)
        flag = obstacle_detector(laser_distance,cell_size)
        print("in main loop")
        print("flag",flag)
        path = []
        if flag == 1:
            robot.setMotion(0.5,-0.7)
            time.sleep(0.5)
        elif flag == 2:
            robot.setMotion(0.5,0.7)
            time.sleep(0.5)
        elif flag == 3:
            robot.setMotion(-1,0)
            time.sleep(5)
        else:
            print("nothing yet")

            goals = goalfinder.find_goals(baye_map,robot_coord)
            print("any goals?",goals)
            if len(goals) > 0:
                path = pick_a_path(baye_map,robot_coord,goals)
                print("any paths?",path)
            else:
                endflag = True       

        if endflag:
            break
        print("step 1 done")    
        # 2. Follow the path     
        # -------Path Module
        for path_num in range(len(path)-1):
            print("In step 2")
            curr_pos = robot.getPosition()
            robot_coord = pos_to_grid(curr_pos['X'], curr_pos['Y'], -60, 60, cell_size)
            laser_scan = robot.getLaser()
            laser_distance = laser_scan['Echoes']
            # 2.0. Break when obstacle exists
            if obstacle_detector(laser_distance,cell_size)!=0:
                break
            # 2.1. Adjust heading direction to next goal on path
            while path_num == 0:
                next_goal = path[1]
                robot_heading = robot.getHeading()
                rcs_angle = RCS_angle_calculation(robot_coord,next_goal,robot_heading)
                robot.setMotion(0,0.8)
                # print("turnning path_num = ",path_num)
                if rcs_angle < 0.3 and rcs_angle > -0.3:
                    print("current point", robot_coord)
                    print("first_goal",next_goal)
                    print("final_angle",rcs_angle)
                    # time.sleep(30)
                    break
            # 2.2. Go to the next goal in the path
            L = dis(robot_coord,path[path_num])
            print("path_num = ",path_num)
            print ("L = ",L)
            while L > 3:
                next_goal = path[path_num]

                # print("current_place",robot_coord)
                print("current_place",robot_coord)
                print("next_goal",next_goal)
                robot_heading = robot.getHeading()
                rcs_angle = RCS_angle_calculation(robot_coord,next_goal,robot_heading)
                # print(robot_heading)
                # print("RCS_angle",rcs_angle)
                
                robot.setMotion(1.5,-rcs_angle*0.4)
                L = dis(robot_coord,path[path_num])
                print("distance",L)
                print("rcs_angle",rcs_angle)
                # -----Start of Map update--------
                curr_pos = robot.getPosition()
                robot_coord = pos_to_grid(curr_pos['X'], curr_pos['Y'], -60, 60, cell_size)
                robot_heading = robot.getHeading()
                laser_angles = robot.getLaserAngles()
                laser_scan = robot.getLaser()
                laser_distance = laser_scan['Echoes']
                print("oercentage:",path_num/len(path))
                points = sensor_model.get_detected_points_coord(robot_coord,robot_heading,laser_angles,laser_distance)
                lines = sensor_model.get_bresenham_lines(robot_coord,points)

                for i in range(len(lines)):
                    bayesian.baye_map_maker(lines[i],0,robot_coord[0],robot_coord[1])

                baye_map.expand_obstacles()
                map.updateMap(baye_map.grid,1.0,robot_coord[0],robot_coord[1])
                # -----End of Map update--------
                if L < 2:
                    break
        # -------Path Module
    map.close()               


            











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

def dis(ps1, ps2):
    x_1, y_1 = ps1[0], ps1[1]
    x_2, y_2 = ps2[0], ps2[1]
    d_x = abs(x_1 - x_2)
    d_y = abs(y_1 - y_2)
    return sqrt(d_x * d_x + d_y * d_y)

if __name__ == '__main__':
    main()