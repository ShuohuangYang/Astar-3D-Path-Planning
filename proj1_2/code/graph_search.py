from heapq import heappush, heappop  # Recommended.
import numpy as np
import math

from flightsim.world import World
from proj1_2.code.occupancy_map import OccupancyMap  # Recommended.


def graph_search(world, resolution, margin, start, goal, astar):
    """
    Parameters:
        world,      World object representing the environment obstacles
        resolution, xyz resolution in meters for an occupancy map, shape=(3,)
        margin,     minimum allowed distance in meters from path to obstacles.
        start,      xyz position in meters, shape=(3,)
        goal,       xyz position in meters, shape=(3,)
        astar,      if True use A*, else use Dijkstra
    Output:
        path,       xyz position coordinates along the path in meters with
                    shape=(N,3). These are typically the centers of visited
                    voxels of an occupancy map. The first point must be the
                    start and the last point must be the goal. If no path
                    exists, return None.
    """

    # While not required, we have provided an occupancy map you may use or modify.
    occ_map = OccupancyMap(world, resolution, margin)
    # Retrieve the index in the occupancy grid matrix corresponding to a position in space.
    start_index = tuple(occ_map.metric_to_index(start))
    goal_index = tuple(occ_map.metric_to_index(goal))
    path = [goal]
    i, j, k = occ_map.map.shape
    g = np.full((i, j, k), np.inf)
    p = np.zeros((i, j, k, 3))
    G = []
    co = 10
    if astar:
        h1 = max(abs(start_index[0] - goal_index[0]) * co, abs(start_index[1] - goal_index[1]) * co,
                 abs(start_index[2] - goal_index[2]) * co)
        # # euclidean
        # h1 = math.sqrt(
        #     (abs(start_index[0] - goal_index[0]) * co) ** 2 + (abs(start_index[1] - goal_index[1]) * co) ** 2 + \
        #     (abs(start_index[2] - goal_index[2]) * co) ** 2)
    else:
        h1 = 0
    g[start_index] = h1
    count3 = 0
    count4 = 0

    heappush(G, (g[start_index], start_index))
    heappush(G, (g[goal_index], goal_index))
    state = np.zeros((i, j, k))  # 0 is not open, 1 is open, 2 is close
    state[start_index] = 1  # add start_index to open
    print("enter loop")
    while np.min(g) < np.inf and not state[goal_index] == 2:  # while there is node not open and the goal is not closed
        u = heappop(G)[1]
        state[u] = 2
        count3 = count3 + 1
        for i in range(u[0] - 1, u[0] + 2):
            for j in range(u[1] - 1, u[1] + 2):
                for k in range(u[2] - 1, u[2] + 2):
                    v = (i, j, k)
                    # determine the distance between voxel
                    if occ_map.is_valid_index(v):
                        if not occ_map.is_occupied_index(v):
                            if not state[v] == 2:  # if neighbot is not closed
                                # calculate maximum distance
                                count4 = count4 + 1
                                if astar:
                                    h = max(abs(goal_index[0] - v[0]) * co, abs(goal_index[1] - v[1]) * co,
                                            abs(goal_index[2] - v[2]) * co)
                                    # euclidean
                                    # h = math.sqrt(
                                    #     (abs(goal_index[0] - v[0]) * co) ** 2 + (abs(goal_index[1] - v[1]) * co) ** 2 \
                                    #     + (abs(goal_index[2] - v[2]) * co) ** 2)
                                else:
                                    h = 0
                                if abs(v[0] - u[0]) + abs(v[1] - u[1]) + abs(v[2] - u[2]) == 1:
                                    c = 10
                                elif abs(v[0] - u[0]) + abs(v[1] - u[1]) + abs(v[2] - u[2]) == 2:
                                    c = 14
                                else:
                                    c = 17
                                d = g[u] + c + h
                                # print("in loop")
                                if state[v] == 1:  # this neighbor is already opened
                                    if g[v] > d:
                                        G.remove((g[v], v))
                                        g[v] = d
                                        p[v] = u
                                        heappush(G, (g[v], v))
                                elif state[v] == 0:  # this neighbor haven't been touched
                                    g[v] = d
                                    p[v] = u
                                    heappush(G, (g[v], v))
                                    state[v] = 1
    print("count1")
    print(count3)
    print("count2")
    print(count4)
    print("out loop")
    temp = state[goal_index]
    f = p[goal_index]
    if (p[goal_index] == (0, 0, 0)).all():  # goal not found
        path = None
    else:
        pointer = goal_index
        while pointer != start_index:
            par = p[pointer]
            path.append(occ_map.index_to_metric_center(par))
            ind1 = int(par[0])
            ind2 = int(par[1])
            ind3 = int(par[2])
            pointer = (ind1, ind2, ind3)

        path.append(start)
        path.reverse()
        print(path)
        path = np.asarray(path)
        print(path.shape)

        print(path[1])
        print(path[1][0])
        print(path[1][1])
        n, d = path.shape
        removeList = []

        for i in range(n-2):
            if path[i+2][0] - path[i+1][0] == path[i+1][0] - path[i][0] and path[i+2][1] - path[i+1][1] == path[i+1][1] - path[i][1] and path[i+2][2] - path[i+1][2] == path[i+1][2] - path[i][2]:
                removeList.append(i+1)
        path = np.delete(path, removeList, axis=0)

        print(path.shape)




        # test for proj1_3
        # row, col = path.shape
        # start = path[0:row - 1, :]
        # end = path[1:row, :]
        # tran = end - start
        # trow,tcol = tran.shape
        # tstart = tran[0:trow-1, :]
        # tend = tran[1:trow, :]
        # dot_prod = tstart.dot(tend.T)
        # se_dot = dot_prod.diagonal()
        # start_abs = np.linalg.norm(tstart, axis=1)
        # end_abs = np.linalg.norm(tend, axis=1)
        # se_abs = np.multiply(start_abs, end_abs)
        # print(se_abs)
        # print(se_dot)
        # se_cos = se_dot / se_abs
        # print(se_cos)
        # n, d = se_cos.shape
        # print(se_cos.shape)
        # print(se_cos)
        # deg = np.degrees(se_cos)
        # degrow, degcol = deg.shape
        # deg_s = deg[0:degrow-1,:]
        # deg_e = deg[1:degrow,:]
        # deg_fin = deg_e - deg_s
        # ind = deg_fin.nonz
        # new_path = []
        # prev = se_cos[0,:]
        # curr = []
        # for i in range(1:1+n):
        #     curr = se_cos[i,:]

    return path
