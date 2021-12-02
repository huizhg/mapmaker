# ------------------------------Done. Dont modify it!  
from math import sin, cos, pi, atan2,floor
class Sensor_model(object):
    def __init__(self, cell_size):
        self.cell_size = cell_size

    def get_detected_points_coord(self,robot_coord,robot_heading,laser_angles,laser_distance):
        point_coord_x = [0]*len(laser_angles)
        point_coord_y = [0]*len(laser_angles)
        for i in range(0, len(laser_angles)): # pi/2 below is used to fix the difference between coordinate system.
            point_coord_x[i] = floor(robot_coord[0]+cos(robot_heading+laser_angles[i]+pi/2)*(laser_distance[i]/self.cell_size)) 
            point_coord_y[i] = floor(robot_coord[1]+sin(robot_heading+laser_angles[i]+pi/2)*(laser_distance[i]/self.cell_size))

            # print(laser_angles[i],laser_distance[i],point_coord_x[i],point_coord_y[i])
            points = [point_coord_x,point_coord_y]
        return points

    def get_bresenham_lines(self,robot_coord,detected_points):
        bresenhamLines = []
        for i in range(0,len(detected_points[0])):
            line = bresenham(robot_coord[0],robot_coord[1],detected_points[0][i],detected_points[1][i])
            bresenhamLines.append(line)
        return bresenhamLines
    
def bresenham(x0, y0, x1, y1):
        line = []
        point = []

        x0=floor(x0)
        x1=floor(x1)
        y0=floor(y0)
        y1=floor(y1)

        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        if (x0<x1):
            sx=1
        else:
            sx=-1
        if (y0<y1):
            sy=1
        else:
            sy=-1
        err = dx+dy

        while (True):
            point=[x0,y0]
            line.append(point)

            if(x0==x1 and y0==y1): 
                return line
            else:
                e2=2*err
            
            if(e2>=dy):
                err+=dy
                x0+=sx
            if(e2<=dx):
                err+=dx
                y0+=sy
# ------------------------------Done. Dont modify it!  


