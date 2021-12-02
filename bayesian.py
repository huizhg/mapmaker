# ------------------------------Done. Dont modify it!  
from math import sqrt,floor

class Bayesian(object):

    def __init__(self, baye_map, cell_size):
        # Definition (p12, Lecture 7)

        self.baye_map = baye_map # Prob grid map

        self.maximum_range = 40/cell_size
        self.beta = 0.5  # Half main lobe angle
        self.p_max = 0.98
        self.R1_range = 1.0

        self.robot_row = 0
        self.robot_col = 0
    
    # Baisc probability (p13-14, Lecture 7)
    def R1_O(self, r, alpha):
        p_temp = ((self.maximum_range - r) / self.maximum_range + (self.beta - abs(alpha)) / self.beta) / 2 * self.p_max
        # print((self.maximum_range - r) / self.maximum_range,(self.beta - abs(alpha)) / self.beta)
        return round(p_temp,7)

    def R1_E(self, r, alpha):
        return 1.0 - self.R1_O(r, alpha)

    def R2_E(self, r, alpha):
        p_temp = ((self.maximum_range - r) / self.maximum_range + (self.beta - abs(alpha)) / self.beta) / 2
        # print((self.maximum_range - r) / self.maximum_range,(self.beta - abs(alpha)) / self.beta)
        return round(p_temp,7)

    def R2_O(self, r, alpha):
        return 1.0 - self.R2_E(r, alpha)
    # Obtain p(Occupied), prior probabilities
    def get_prob(self, cell):
        return self.baye_map[cell[0]][cell[1]]

    # get the region of grid
    def get_region(self, cell_online, cell_readout):
        # if abs(cell_online[0]-cell_readout[0])<2 or abs(cell_online[1]-cell_readout[1])<2:
        #     return 1
        # else:
        #     return 2

        d_robot_readout = sqrt((cell_readout[0]-self.robot_row)**2 + (cell_readout[1]-self.robot_col)**2)
        # print("D",d_robot_readout)
        d_robot_cell_online = sqrt((cell_online[0]-self.robot_row)**2 + (cell_online[1]-self.robot_col)**2)
        # print("d2",d_robot_cell_online)
        d_cell_online_readout = sqrt((cell_readout[0]-cell_online[0])**2 + (cell_readout[1]-cell_online[1])**2)
        # print("d3",d_cell_online_readout)
        if d_robot_cell_online > (d_robot_readout+self.R1_range):
            return 3 # out of range
        elif d_cell_online_readout < self.R1_range or (d_robot_cell_online > (d_robot_readout-self.R1_range)):
            return 1 # In the range of R1.
        # elif d_robot_cell_online < (d_robot_readout-self.R1_range):
        #     return 2 # Close to robot 
        else:
            return 2 
     

    # obtain OCCUPANCY PROBABILITIES (p18, Lecture 7)
    def get_occ_prob(self, region, alpha, cell_online, cell_readout):
        # r = d_robot_cell_online    
        r = sqrt((cell_online[0]-self.robot_row)**2 + (cell_online[1]-self.robot_col)**2) 
        # print("r: ",r)
        if region == 1:
            P_O = self.get_prob(cell_online)
            P_sO = self.R1_O(r,alpha)
            P_E = 1 - P_O
            P_sE = 1 - P_sO
            # print("P_sO: ",P_sO)
            # print("P_O: ", P_O)
            # print("P_sE: ",P_sE)
            # print("P_E:", P_E)
            # print("P_Os: ",P_sO*P_O/(P_sO*P_O+P_sE*P_E))
            return P_sO*P_O/(P_sO*P_O+P_sE*P_E)

        elif region == 2:
            P_O = self.get_prob(cell_online)
            P_sO = self.R2_O(r,alpha)
            P_E = 1.0 - P_O
            P_sE = 1.0 - P_sO
            # print("P_sO: ",P_sO)
            # print("P_O: ", P_O)
            # print("P_sE: ",P_sE)
            # print("P_E:", P_E)
            # print("P_Os: ",P_sO*P_O/(P_sO*P_O+P_sE*P_E))
            return P_sO*P_O/(P_sO*P_O+P_sE*P_E)

        else:
            pass # Do not update
# --------------------united function:
    # Make map with each line 
    def baye_map_maker(self, line, alpha, robot_row, robot_col):
        self.robot_row = floor(robot_row)
        self.robot_col = floor(robot_col)
        

        cell_readout = line[len(line) - 1]
        # print("readout",cell_readout)
        for i in range (0, len(line)):
            cell_online = line[i]
            region = self.get_region(cell_online,cell_readout)
            
            if cell_online[0] < 120 and cell_online[0] > 0:
                if cell_online[1] <120 and cell_online[1] > 0:
                    p = self.get_occ_prob(region,alpha,cell_online,cell_readout)
                    self.baye_map[cell_online[0]][cell_online[1]] = p
            # print(cell_online)
            # print(region)
            # print(p)
            # print("\n")
# ------------------------------Done. Dont modify it!  
 

