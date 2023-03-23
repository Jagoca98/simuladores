#!/usr/bin/env python
# https://towardsdatascience.com/reinforcement-learning-with-openai-d445c2c687d2

import termios
import tty
import sys
import gymnasium as gym
import gymnasium_csv
from gymnasium_csv.wrappers import BoxToDiscreteObservation
import math
import time

import numpy as np


init_x = 2
init_y = 2
end_x = 6
end_y = 4

def index2map(index):
    """!@brief Convert from index to map coordinates.
    This function converts the index position of the charMap cell to the (x,y) coordinates of the map cell.
    @param index The index of the charMap cell.
    @return (x, y) The coordinates (x, y) of the map cell. 
    """
    return(index%(SIZE_X), math.trunc(index/(SIZE_X)))

def map2index(x, y):
    """!@brief Convert from map coordinates to index.
    This function converts the (x,y) coordinates of the map to the index position of the charMap.
    @param x The x coordinate of a map cell.
    @param y The y coordinate of a map cell.
    @return index The index of the charMap cell.
    """
    return(SIZE_X * y + x)

def update_value(index, value):
    """!@brief Update the value of a cell.
    This function updates the index cell of a charMap with the value value.
    @param index The index of the charMap cell.
    @param value The value of the map cell.
    """
    charMap[index] = value;

def nhood4(index):
    """!@brief Search 4 neighbourgs
    Determine 4-connected neighbourhood of an input cell, checking for map edges
    @param index Input cell index
    @return 4-neighbour cell indexes
    """
    out = []
    if(index > SIZE_X * SIZE_Y -1):
        print("Evaluating nhood out of the map")
        return out
    if(index % SIZE_X > 0):
        out.append(index - 1)
    if(index % SIZE_X < SIZE_X - 1):
        out.append(index + 1)
    if(index>= SIZE_X):
        out.append(index - SIZE_X)
    if(index < SIZE_X * (SIZE_Y - 1)):
        out.append(index + SIZE_X)
    return out

def nhood8(index):
    """!@brief Search 8 neighbourgs
    Determine 8-connected neighbourhood of an input cell, checking for map edges
    @param index Input cell index
    @return 8-neighbour cell indexes
    """
    out = nhood4(index)
    if(index > SIZE_X * SIZE_Y - 1):
        return out
    if(index % SIZE_X > 0 and index >= SIZE_X):
        out.append(index - 1 - SIZE_X)
    if(index % SIZE_X > 0 and index < SIZE_X * (SIZE_Y - 1)):
        out.append(index - 1 + SIZE_X)
    if(index % SIZE_X < SIZE_X -1 and index >= SIZE_X):
        out.append(index + 1 - SIZE_X)
    if(index % SIZE_X < SIZE_X - 1 and index < SIZE_X * (SIZE_Y - 1)):
        out.append(index + 1 + SIZE_X)
    return out

def direction(node, parentnode):
    UP = 0
    UP_RIGHT = 1
    RIGHT = 2
    DOWN_RIGHT = 3
    DOWN = 4
    DOWN_LEFT = 5
    LEFT = 6
    UP_LEFT = 7
    if((node.x-parentnode.x == 0) and (node.y-parentnode.y == -1)):
        direction = DOWN
    elif((node.x-parentnode.x == 0) and (node.y-parentnode.y == 1)):
        direction = UP
    elif((node.x-parentnode.x == 1) and (node.y-parentnode.y == 0)):
        direction = LEFT
    elif((node.x-parentnode.x == -1) and (node.y-parentnode.y == 0)):
        direction = RIGHT
    elif((node.x-parentnode.x == 1) and (node.y-parentnode.y == -1)):
        direction = DOWN_LEFT
    elif((node.x-parentnode.x == -1) and (node.y-parentnode.y == -1)):
        direction = DOWN_RIGHT
    elif((node.x-parentnode.x == 1) and (node.y-parentnode.y == 1)):
        direction = UP_LEFT
    elif((node.x-parentnode.x == -1) and (node.y-parentnode.y == 1)):
        direction = UP_RIGHT        
    return direction

class Node:
    def __init__(self, x, y, myId, parentId, distance):
        self.x = x
        self.y = y
        self.myId = myId
        self.parentId = parentId
        self.distance = distance
    def dump(self):
        print("---------- x "+str(self.x)+\
                         " | y "+str(self.y)+\
                         " | id "+str(self.myId)+\
                         " | parentId "+str(self.parentId))

env_raw = gym.make('gymnasium_csv-v0',
                   render_mode='human',  # "human", "text", None
                   inFileStr='../assets/map1.csv',
                   initX=init_y,
                   initY=init_x,
                   goalX=end_y,
                   goalY=end_x)
env = BoxToDiscreteObservation(env_raw)
env.reset()
env.render()


SIM_PERIOD_MS = 500.0


FILE_NAME = "../assets/map1.csv"

charMap = []
with open(FILE_NAME) as f:
            line = f.readline()
            while line:
                charLine = line.strip().split(',')
                charMap.append(charLine)
                line = f.readline() 

SIZE_X = len(charLine)
SIZE_Y = len(charMap)

i_start = map2index(init_x,init_y)
# update_value(i_start, 3)

i_goal = map2index(end_x, end_y)
# update_value (i_goal, 4)

charMap = np.concatenate(charMap)
distance = max(abs(init_x-end_x), abs(init_y-end_y))
init = Node(init_x, init_y, 0, map2index(init_x, init_y), distance)
goal = Node(end_x, end_y, -2, map2index(end_x, end_y), 0)

goalParentId = -1
visited_flag = np.zeros(SIZE_X * SIZE_Y)
bfs = []
nodes = []
path = []
bfs.append(init)
nodes.append(init)
done = False
ok = False

while (not done):
    idx = bfs[0]
    index = map2index(idx.x, idx.y)
    bfs.pop(0)
    # print(len(bfs))

    for n in nhood8(index):
        if (not visited_flag[n]):
                visited_flag[n] = True
                if(charMap[n] == '0'):
                    x_tmp, y_tmp = index2map(n)
                    distance = max(abs(end_x - x_tmp), abs(end_y - y_tmp))  
                    node_tmp = Node(x_tmp, y_tmp, n, index, distance)
                    bfs.append(node_tmp)
                    nodes.append(node_tmp)
                    # print(n, i_goal, done)
                if(n == i_goal):
                    done = True
                    x_tmp, y_tmp = index2map(n)
                    distance = max(abs(end_x - x_tmp), abs(end_y - y_tmp))  
                    node_tmp = Node(x_tmp, y_tmp, goalParentId, index, distance)
                    bfs.append(node_tmp)
                    nodes.append(node_tmp)
                    # print("Golaso mi nino oleoleole")
                # elif (charMap[n] == '4'):
                #     done = True
                #     x_tmp, y_tmp = index2map(n)
                #     distance = max(abs(end_x - x_tmp), abs(end_y - y_tmp))  
                #     node_tmp = Node(x_tmp, y_tmp, goalParentId, index, distance)
                #     bfs.append(node_tmp)
                #     nodes.append(node_tmp)
                #     print("Golaso mi nino oleoleole")
    bfs = sorted(bfs, key=lambda x: x.distance) ## Sort bfs nodes by distance to the goal

# input('')

while not ok:
    # print('s')
    for node in nodes:
        if(node.myId == goalParentId):
            
            # if(charMap[map2index(node.x, node.y)] == '2'):
                # update_value(map2index(node.x, node.y), 5)
            # node.dump()
            path.append(node)
            goalParentId = node.parentId
            if(goalParentId == map2index(init_x, init_y)):
                # print("%%%%%%%%%%%%%%%%%")
                ok = True
# path.insert(0, goal)
# print(path[0].x, path[0].y)

# path = sorted(path, reverse=True)

for i in range(len(path)-1, 0, -1):
    path[i].dump()
    direction_var = direction(path[i], path[i-1])
    # print(direction_var)
    observation, reward, terminated, truncated, info = env.step(direction_var)
    env.render()
    print("observation: " + str(observation)+", reward: " + str(reward) + ", terminated: " +
          str(terminated) + ", truncated: " + str(truncated) + ", info: " + str(info))
    time.sleep(SIM_PERIOD_MS/1000.0)
print("--------------")
input('')