from math import sin, cos, atan2, pi
from astar_path_fixed import astar_path
def RCS_angle_calculation(CP,GP,Heading_value):
    Heading_value = Heading_value + pi/2
    CP_x = CP[0]
    CP_y = CP[1]

    GP_x = GP[0]
    GP_y = GP[1]

    RCS_x = (GP_x - CP_x) * cos(Heading_value) + (GP_y - CP_y) * sin(Heading_value)
    RCS_y = (GP_y - CP_y) * cos(Heading_value) - (GP_x - CP_x) * sin(Heading_value)
    return (-atan2(RCS_y, RCS_x))

def obstacle_detector(laser_distance,cell_size):
    """
    1 for left, 2 for right, 0 for nothing.
    """
    left_sign = 0
    right_sign = 0
    for angle_num, dis in enumerate(laser_distance):
        if dis/cell_size < 1.5*cell_size:
            if angle_num < 135 : 
                # object at right
                right_sign = 2
            else:
                # object at left
                left_sign = 1
    return left_sign+right_sign

def pick_a_path(map, start, goals):
    for goal in goals:
        print("stack in where?",goal)
        path = []
        # start_x, start_y = start[0], start[1]
        # goal_x, goal_y = goal[0], goal[1]
        # if abs(start_x-goal_x)** 2 + abs(start_y-goal_y)**2 > 1600:
        #     goal = (0,0)
        #     path = astar_path(map, start, goal)
        # else:
        path = astar_path(map, start, goal)
        if len(path) >= 4:
            return path

    return []

