import sys
import os
import numpy as np
import matplotlib.pyplot as plt

MAP_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), '3-map/map.npy')

### START CODE HERE ###
# This code block is optional. You can define your utility function and class in this block if necessary.
def valid_pos(world_map, pos):
    if pos[0] < 0 or pos[1] < 0 or pos[0] >= world_map.shape[0] or pos[1] >= world_map.shape[1]:
        return False
    elif world_map[pos[0]][pos[1]] == 1:
        return False
    else:
        return True

def cost(pos1, pos2):
    return 1

#Manhattan Distance
def h(now_pos, goal_pos):
    return abs(now_pos[0] - goal_pos[0]) + abs(now_pos[1] - goal_pos[1])

class Node:
    def __init__(self, x, y, g, h, pre):
        self.x = x
        self.y = y
        self.g = g
        self.h = h
        self.pre = pre

    def get_nearby(self, world_map, goal_pos):
        nearby = []
        checklist = [(self.x+1, self.y), (self.x-1, self.y), (self.x, self.y+1), (self.x, self.y-1)]
        for pos in checklist:
            if valid_pos(world_map, pos):
                nearby.append(Node(pos[0], pos[1], self.g+cost((self.x, self.y), pos), h(pos, goal_pos), self))
            else:
                continue
        return nearby



###  END CODE HERE  ###


def A_star(world_map, start_pos, goal_pos):
    """
    Given map of the world, start position of the robot and the position of the goal, 
    plan a path from start position to the goal using A* algorithm.

    Arguments:
    world_map -- A 120*120 array indicating map, where 0 indicating traversable and 1 indicating obstacles.
    start_pos -- A 2D vector indicating the start position of the robot.
    goal_pos -- A 2D vector indicating the position of the goal.

    Return:
    path -- A N*2 array representing the planned path by A* algorithm.
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

    #use attribute pre to get path
    while (nowN.x, nowN.y) != (startN.x, startN.y):
        path.insert(0, [nowN.x, nowN.y])
        nowN = nowN.pre
    path.insert(0, [nowN.x, nowN.y])

    ###  END CODE HERE  ###
    return path





if __name__ == '__main__':

    # Get the map of the world representing in a 120*120 array, where 0 indicating traversable and 1 indicating obstacles.
    map = np.load(MAP_PATH)

    # Define goal position of the exploration
    goal_pos = [100, 100]

    # Define start position of the robot.
    start_pos = [10, 10]

    # Plan a path based on map from start position of the robot to the goal.
    path = A_star(map, start_pos, goal_pos)

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

  
