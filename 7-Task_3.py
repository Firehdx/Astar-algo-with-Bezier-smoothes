import sys
import os
import numpy as np
import matplotlib.pyplot as plt

MAP_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), '3-map/map.npy')


### START CODE HERE ###
# This code block is optional. You can define your utility function and class in this block if necessary.

#parameters
Safe_Distance = 2
Penalty_Factor = 1
Steer_Factor = 1
Turning_R = 4
InterpolationNum = 100



## PATH GENERATING ##
def in_map(world_map, pos):
    return not (pos[0] < 0 or pos[1] < 0 or pos[0] >= world_map.shape[0] or pos[1] >= world_map.shape[1])

def valid_pos(world_map, pos):
    if not in_map(world_map, pos):
        return False
    elif world_map[pos[0]][pos[1]] == 1:
        return False
    else:
        return True

#there's a possibility of runnning in a circle
def change_dir(pos0, pos1, pos2):
    vec1 = (pos1[0]-pos0[0], pos1[1]-pos0[1])
    vec2 = (pos2[0]-pos1[0], pos2[1]-pos1[1])
    return vec1 != vec2

#consider 8 dirs and the cost of turning, pos1 is the current position
def cost(pos0, pos1, pos2):
    s = 0
    if change_dir(pos0, pos1, pos2):
        s = Steer_Factor
    if pos1[0] == pos2[0] or pos1[1] == pos2[1]:
        return 1 + s
    else:
        return 1.4 + s

#penalty term of too close to obstacles
def obstacle_penalty(now_pos, world_map):
    p = 0
    dl = (now_pos[0]-Safe_Distance, now_pos[1]-Safe_Distance)
    dr = (now_pos[0]+Safe_Distance, now_pos[1]-Safe_Distance)
    ul = (now_pos[0]-Safe_Distance, now_pos[1]+Safe_Distance)
    ur = (now_pos[0]+Safe_Distance, now_pos[1]+Safe_Distance)
    if in_map(world_map, dl) and in_map(world_map, ur):
        p = Penalty_Factor * np.sum(map[dl[0]:dr[0]+1, dl[1]:ul[1]+1])
    return p

def Manhattan_Distance(now_pos, goal_pos):
    return abs(now_pos[0] - goal_pos[0]) + abs(now_pos[1] - goal_pos[1])

def Euclidean_Distance(now_pos, goal_pos):
    return ((now_pos[0] - goal_pos[0])**2 + (now_pos[1] - goal_pos[1])**2)**0.5

#expected here2goal cost function
def h(now_pos, goal_pos, world_map):
    return Manhattan_Distance(now_pos, goal_pos) + obstacle_penalty(now_pos, world_map)

class Node:
    def __init__(self, x, y, g, h, pre):
        self.x = x
        self.y = y
        self.g = g
        self.h = h
        self.pre = pre

    def get_nearby(self, world_map, goal_pos):
        nearby = []
        checklist = [(self.x+1, self.y), (self.x-1, self.y), (self.x, self.y+1), (self.x, self.y-1), 
                     (self.x+1, self.y+1), (self.x-1, self.y+1), (self.x+1, self.y-1), (self.x-1, self.y-1)]
        for pos in checklist:
            if valid_pos(world_map, pos):
                if self.pre == None:
                    last_pos = (self.x, self.y)
                else:
                    last_pos = (self.pre.x, self.pre.y)
                nearby.append(Node(pos[0], pos[1], self.g+cost(last_pos, (self.x, self.y), pos), h(pos, goal_pos, world_map), self))
            else:
                continue
        return nearby



## PATH SMOOTHES##
#Here we smoothes the generated path (instead of generate a smooth path in a single process)
#We use partial Bezier method, i.e. use Bezier curves to fit the points at the turning
def factorial(n):
    result = 1
    for i in range(1, n+1):
        result *= i
    return result

class Bezier:
    def __init__(self, Points, InterpolationNum):
        self.order = Points.shape[0]-1
        self.num = InterpolationNum
        self.pointsNum = Points.shape[0]
        self.Points = Points
        
    # get all interpolation points of Bezeir curve
    def getBezierPoints(self):
        PB=np.zeros((self.pointsNum, 2))
        pis =[]
        for u in np.arange(0, 1 + 1/self.num, 1/self.num):
            for i in range(0, self.pointsNum):
                PB[i] = (factorial(self.order)/(factorial(i)*factorial(self.order - i)))*(u**i)*(1-u)**(self.order - i)*self.Points[i]
            pi = sum(PB).tolist()
            pis.append(pi)
        return np.array(pis)

#find the turning points in the path (as well as its index)
def get_turnings(path):
    turnings = []
    turnings_index = []
    for i in range(1, len(path)-1):
        if change_dir(path[i-1], path[i], path[i+1]):
            turnings_index.append(i)
            turnings.append(path[i])
    return turnings, turnings_index

#return a list of curves which consist of lists of control points, use an aux func
def control_points(path):
    turnings, turnings_index = get_turnings(path)
    #the index of current turning point in the turning point list
    index = 0
    result = []
    while index != len(turnings_index):
        cp_set, index = control_points_aux(path, turnings_index, [path[turnings_index[index]]], index, False)
        result.append(cp_set)
        index += 1
    return result

#return a list of control points of current curve as well as the index of the current turning point to use a recursion
#find the point at the +-Turning_R area of the turning point (a turning area) to get a group of control points of Bezier curve
#if two turning areas cross then take their union
def control_points_aux(path, turnings_index, cp_set, index, extending):
    index_in_path = turnings_index[index]

    #boundary condition
    #left
    if extending == False:
        if index_in_path < Turning_R:
            cp_set.insert(0, path[0])
        else:
            cp_set.insert(0, path[index_in_path - Turning_R])
    #right
    if index_in_path + 2*Turning_R > len(path) - 1:
        #last turning point
        if index == len(turnings_index) - 1:
            if index_in_path + Turning_R > len(path) - 1:
                cp_set.append(path[-1])
            else:
                cp_set.append(path[index_in_path + Turning_R])
            return cp_set, index
        #not the last
        else:
            cp_set.append(path[turnings_index[index + 1]])
            index += 1
            return control_points_aux(path, turnings_index, cp_set, index, True)
    #the last turning point but not at the boundary
    if index == len(turnings_index) - 1:
        cp_set.append(path[index_in_path + Turning_R])
        return cp_set, index
        
    #if there's a new turning at this turning area, append it with its turning area
    if index_in_path + 2*Turning_R > turnings_index[index + 1]:
        cp_set.append(path[turnings_index[index + 1]])
        index += 1
        return control_points_aux(path, turnings_index, cp_set, index, True)
    #with no new turnings at current turning area, return
    else:
        cp_set.append(path[index_in_path + Turning_R])
        return cp_set, index

def path_smoother(path):
    curves = []
    cps = control_points(path)
    for cp in cps:
        points = np.array(cp)
        bz = Bezier(points,InterpolationNum)
        curves.append(bz.getBezierPoints())
    smooth_path = curves[0]
    for i in range(1, len(curves)):
        smooth_path = np.append(smooth_path, curves[i], axis=0)
    smooth_path = smooth_path.tolist()
    start_index = path.index(smooth_path[0])
    smooth_path = path[:start_index] + smooth_path
    return smooth_path

###  END CODE HERE  ###



def self_driving_path_planner(world_map, start_pos, goal_pos):
    """
    Given map of the world, start position of the robot and the position of the goal, 
    plan a path from start position to the goal.

    Arguments:
    world_map -- A 120*120 array indicating map, where 0 indicating traversable and 1 indicating obstacles.
    start_pos -- A 2D vector indicating the start position of the robot.
    goal_pos -- A 2D vector indicating the position of the goal.

    Return:
    path -- A N*2 array representing the planned path.
    """

    ### START CODE HERE ###
    path = []
    openset = []
    closeset = []

    #init
    startN = Node(start_pos[0], start_pos[1], 0, 0, None)
    closeset.append(startN)
    nowN = startN

    while [nowN.x, nowN.y] != goal_pos:
        #append the nearby points to the openset
        nearN = nowN.get_nearby(world_map, goal_pos)
        for newN in nearN:
            if newN in closeset:
                continue
            exist_in_openset = False
            for node in openset:
                if (node.x, node.y) == (newN.x, newN.y):
                    exist_in_openset = True
                    if node.g > newN.g:
                        node = newN
            if not exist_in_openset:
                openset.append(newN)

        #find the new point of the least cost
        now_index = np.argmin([i.g + i.h for i in openset])
        nowN = openset.pop(now_index)
        closeset.append(nowN)

    #use the attribute pre to get path
    while (nowN.x, nowN.y) != (startN.x, startN.y):
        path.insert(0, [nowN.x, nowN.y])
        nowN = nowN.pre
    path.insert(0, [nowN.x, nowN.y])

    path = path_smoother(path)
  

    ###  END CODE HERE  ###
    return path




if __name__ == '__main__':

    # Get the map of the world representing in a 120*120 array, where 0 indicating traversable and 1 indicating obstacles.
    map = np.load(MAP_PATH)

    # Define goal position
    goal_pos = [100, 100]

    # Define start position of the robot.
    start_pos = [10, 10]

    # Plan a path based on map from start position of the robot to the goal.
    path = self_driving_path_planner(map, start_pos, goal_pos)
    #print(path)
    
    # Visualize the map and path.
    obstacles_x, obstacles_y = [], []
    for i in range(120):
        for j in range(120):
            if map[i][j] == 1:
                obstacles_x.append(i)
                obstacles_y.append(j)

    path_x, path_y = [], []
    for path_node in path:
        path_x.append(path_node[0])
        path_y.append(path_node[1])

    plt.plot(path_x, path_y, "-r")
    plt.plot(start_pos[0], start_pos[1], "xr")
    plt.plot(goal_pos[0], goal_pos[1], "xb")
    plt.plot(obstacles_x, obstacles_y, ".k")
    plt.grid(True)
    plt.axis("equal")
    plt.show()
    