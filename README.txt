Graph_search

1.The main structure for Dijkstra and A* is almost the same, except that A* use heuristic function, Diagonal distance, also called Chebyshev distance, since we are only moving in parallel or diagonal direction. This heuristic is just taking the maximum of the difference among coordinates. 
2.g is a 3darray used to store all the cost for the nodes, initialized to be 0.
3.p is a 4darray used to store all the parent index for the nodes, initialized to be all infinity.
4.G is a heapq which is all the open nodes with their cost. (cost,(index))
5.State is a 3darray used to store the state of nodes. Initialize to be 0. 0 means this node have not been touched yet. 1 means this is an open node. 2 means this is a close node.
6.The start_index and goal_index with its cost are heappushed into G at the initialization.
7.In the while loop, node with smallest cost is popped. Its neighbor is checked one by one, if state is 0, heappush. If state is 1, check if update cost is needed. Parents are updated as well.
8.A* and Dijkstra is swiching by the “if astar” condition.
