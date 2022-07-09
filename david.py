from timeit import repeat
import matplotlib.pyplot as plt
from collections import deque
import networkx as nx
import time 
import numpy as np
from matplotlib import animation, rc
from IPython.display import HTML
from matplotlib.animation import FuncAnimation

class Graph:
    # example of adjacency list (or rather map)
    # adjacency_list = {
    # 'A': [('B', 1), ('C', 3), ('D', 7)],
    # 'B': [('D', 5)],
    # 'C': [('D', 12)]
    # }

    def __init__(self, adjacency_list):
        self.adjacency_list = adjacency_list

    def get_neighbors(self, v):
        return self.adjacency_list[v]

    # heuristic function with equal values for all nodes
    def h(self, n):
        H = {
            'A': 1,
            'B': 1,
            'C': 1,
            'D': 1,
            'E': 1,
            'F': 1,
        }

        return H[n]

    def a_star_algorithm(self, start_node, stop_node):
        # open_list is a list of nodes which have been visited, but who's neighbors
        # haven't all been inspected, starts off with the start node
        # closed_list is a list of nodes which have been visited
        # and who's neighbors have been inspected
        open_list = set([start_node])
        closed_list = set([])

        # g contains current distances from start_node to all other nodes
        # the default value (if it's not found in the map) is +infinity
        g = {}

        g[start_node] = 0

        # parents contains an adjacency map of all nodes
        parents = {}
        parents[start_node] = start_node

        while len(open_list) > 0:
            n = None

            # find a node with the lowest value of f() - evaluation function
            for v in open_list:
                if n == None or g[v] + self.h(v) < g[n] + self.h(n):
                    n = v;

            if n == None:
                print('Path does not exist!')
                return None

            # if the current node is the stop_node
            # then we begin reconstructin the path from it to the start_node
            if n == stop_node:
                reconst_path = []

                while parents[n] != n:
                    reconst_path.append(n)
                    n = parents[n]

                reconst_path.append(start_node)

                reconst_path.reverse()

                # print('Path found: {}'.format(reconst_path))
                return reconst_path

            # for all neighbors of the current node do
            for (m, weight) in self.get_neighbors(n):
                # if the current node isn't in both open_list and closed_list
                # add it to open_list and note n as it's parent
                if m not in open_list and m not in closed_list:
                    open_list.add(m)
                    parents[m] = n
                    g[m] = g[n] + weight

                # otherwise, check if it's quicker to first visit n, then m
                # and if it is, update parent data and g data
                # and if the node was in the closed_list, move it to open_list
                else:
                    if g[m] > g[n] + weight:
                        g[m] = g[n] + weight
                        parents[m] = n

                        if m in closed_list:
                            closed_list.remove(m)
                            open_list.add(m)

            # remove n from the open_list, and add it to closed_list
            # because all of his neighbors were inspected
            open_list.remove(n)
            closed_list.add(n)

        print('Path does not exist!')
        return None

adjac_lis = {
    'A': [('B', 10), ('C', 12), ('D', 5)],
    'B': [('A', 10), ('E',11)],
    'C': [('A', 12), ('D', 6),('E', 11)],
    'D': [('A',5), ('C',6),('F',14)],
    'E': [('B',11), ('C',11)],
    'F': [('C',8), ('D',14)]
}

graph1 = Graph(adjac_lis)

def state_to_color(s):
    if s==0:
        return [0.9,0.9,0.9] #unvisited
    if s==1:
        return [0.5,0.5,1] #visited
    return [1,0.5,0.5] #visiting

def update(t, n, nodes, order, visited):
    nc = [[0.9,0.9,0.9]]*n #node color
    for i in range(len(order)):
        nc[i] = state_to_color(0)
        if i==order[i]:
            nc[i] = state_to_color(2)
        elif visited[i]:
            nc[i] = state_to_color(1)
    visited[t]=True  
    nodes.set_color(nc)
    return nodes,

G = nx.Graph()
G.add_nodes_from(['A','B','C','D','E','F'])
G.add_edge('A','B',weight= 10)
G.add_edge('A','C',weight= 12)
G.add_edge('A','D',weight= 5)
G.add_edge('B','A',weight= 10)
G.add_edge('B','E',weight= 11)
G.add_edge('C','A',weight= 12)
G.add_edge('C','D',weight= 6)
G.add_edge('C','E',weight= 11)
G.add_edge('D','A',weight= 5)
G.add_edge('D','C',weight= 6)
G.add_edge('D','F',weight= 14)
G.add_edge('E','B',weight= 11)
G.add_edge('E','C',weight= 11)
G.add_edge('F','B',weight= 11)
G.add_edge('F','D',weight= 14)

n = G.number_of_nodes()
pos=nx.spring_layout(G)

nc = [[0.9,0.9,0.9]]*n #node color
fig=plt.figure(figsize=(7,7))

nodes = nx.draw_networkx_nodes(G,pos,node_color=nc,node_size=400) #layout of nodes
edges = nx.draw_networkx_edges(G,pos) #layout of edges

nx.draw(G,pos,with_labels=True, font_weight='bold')
labels = nx.get_edge_attributes(G,'weight')
nx.draw_networkx_edge_labels(G,pos,edge_labels=labels)

order = graph1.a_star_algorithm('A', 'F')
visited = [False]*n

anim = FuncAnimation(fig, update, fargs = (n, nodes, order, visited), interval=400,frames=len(order) ,save_count=len(order), blit=True ,repeat=True)
plt.show()

