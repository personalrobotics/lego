import random
import numpy as np

import math

def get_rval(x):
    tmp = random.randint(0,99)
    if tmp%2:
        return random.randint(0,x-1)
    else:
        return random.randint(x+1,9)

def get_rval2(x):
    x1 = get_rval(x)
    x2 = get_rval(x)
    while abs(x1-x2)==1:
        x1 = get_rval(x)
        x2 = get_rval(x)
    return x1, x2

def get_random_occ_grid():
    row = random.randint(2,7)
    col = random.randint(2,7)

    rc1, rc2 = get_rval2(col)
    cr1, cr2 = get_rval2(row)

    occ_grid = np.ones((10,10))
    
    for i in range(10):
        if not (i==rc1 or i==rc2):
            occ_grid[row,i] = 0

        if not (i==cr1 or i==cr2):
            occ_grid[i,col] = 0
    
    return occ_grid, row, col

def state_to_numpy(state):
    strlist = state.split()
    val_list = [float(s) for s in strlist]
    return np.array(val_list)

def numpy_to_state(array):
    state = ""
    for i in range(len(array)):
        state += str(array[i])+" "
    return state

def is_point_free(conf, occ_g, row, col, inc = 0):
    px, py = conf[0], conf[1]
    lx, ly = px-inc, py
    rx, ry = px+inc, py
    ux, uy = px, py+inc
    dx, dy = px, py-inc
    pc = []
    if(int(10*px)==row):
            pc = [(px, py), (ux, uy), (dx, dy)]
    if(int(10*py)==col):
            pc = [(px, py), (lx, ly), (rx, ry)]
    for p in pc:
        x, y = int(10*p[0]), int(10*p[1])
        x = max(min(x, 9),0)
        y = max(min(y, 9),0)
        if(not occ_g[x][y]):
            return 0
    return 1

def is_edge_free(node1_pos, node2_pos, occ_g, row, col, EDGE_DISCRETIZATION = 20, inc = 0.035):
    diff = node2_pos - node1_pos
    step = diff/EDGE_DISCRETIZATION

    for i in range(EDGE_DISCRETIZATION+1):
        nodepos = node1_pos + step*i
        if(not is_point_free(nodepos, occ_g, row, col, inc)):
            return 0
    return 1

def get_valid_start_goal(dense_G, occ_g, row, col, inc):
    start_n = random.choice(list(dense_G.nodes()))
    goal_n = random.choice(list(dense_G.nodes()))

    start = state_to_numpy(dense_G.node[start_n]['state'])
    goal = state_to_numpy(dense_G.node[goal_n]['state'])

    while is_edge_free(start, goal, occ_g, row, col, EDGE_DISCRETIZATION = 100, inc = inc) or not (is_point_free(start, occ_g, row, col, inc) and is_point_free(goal, occ_g, row, col, inc)):
        start_n = random.choice(list(dense_G.nodes()))
        goal_n = random.choice(list(dense_G.nodes()))

        start = state_to_numpy(dense_G.node[start_n]['state'])
        goal = state_to_numpy(dense_G.node[goal_n]['state'])

    return start_n, goal_n

def calc_weight_states(s1, s2):
    config1 = state_to_numpy(s1)
    config2 = state_to_numpy(s2)
    return math.sqrt(float(np.sum((config2-config1)**2)))