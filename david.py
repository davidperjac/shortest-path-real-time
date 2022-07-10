from matplotlib.animation import FuncAnimation
from matplotlib import animation
import matplotlib.pyplot as plt
import networkx as nx
import random as rd
import time 

class Graph:
    def __init__(self, adjacency_list):
        self.adjacency_list = adjacency_list

    def get_neighbors(self, v):
        return self.adjacency_list[v]

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

#ANIMATE FUNCTIONS

def state_to_color():
    return [1,0.5,0.5] #visiting

def update(t, n, nodes, order, visited,G,pos):
    nc = [[0.9,0.9,0.9]]*n
    for i in range(n) : 
        for node in G: 
            if (order[t] == node ) :
                nc[list(G.nodes).index(node)] = state_to_color()
                visited[list(G.nodes).index(node)] = True
                if t  != len(order) - 1: 
                    G[node][order[t+1]]['weight']+=20
                    G[node][order[t+1]]['color']='red'
                    if t != 0:
                        weights = list( map( lambda x: x/2 , nx.get_edge_attributes(G,'weight').values() ) )
                        colors = nx.get_edge_attributes(G,'color').values()
                        labels = nx.get_edge_attributes(G,'weight')
                        nx.draw(G,pos,with_labels=True, font_weight='bold',font_color="white",width=weights, node_color='black', node_size=250,edge_color=colors)
                        nx.draw_networkx_edge_labels(G,pos,edge_labels=labels,font_size=15,with_labels = True)
            elif visited[i]:
                nc[i] = state_to_color()
    nodes.set_color(nc)
    return nodes

def createVisualGraph(graph) :
    G = nx.Graph()
    adjacency_list = graph.adjacency_list
    G.add_nodes_from(list(adjacency_list.keys()))
    for node,edges in adjacency_list.items():
        for edge in edges: 
            G.add_edge(node,edge[0],weight=edge[1],color='black')
    return G

def draw(G,pos,weights) :
    nx.draw(G,pos,with_labels=True, font_weight='bold',font_color="white",width=weights, node_color='black', node_size=250)
    labels = nx.get_edge_attributes(G,'weight')
    nx.draw_networkx_edge_labels(G,pos,edge_labels=labels,font_size=15,with_labels = True)

def animateGraph (graph,order,obstruccion,affectedNode) :
    visualGraph = createVisualGraph(graph)

    total_nodes = visualGraph.number_of_nodes()
    pos=nx.spring_layout(visualGraph)

    node_color = [[0.9,0.9,0.9]]*total_nodes 
    fig=plt.figure(figsize=(7,7))

    nodes = nx.draw_networkx_nodes(visualGraph,pos,node_color=node_color,node_size=400) 
    weights = list( map( lambda x: x/2 , nx.get_edge_attributes(visualGraph,'weight').values() ) )

    draw(visualGraph,pos,weights)
    visited = [False]*total_nodes

    anim = FuncAnimation(fig, update, fargs = (total_nodes, nodes, order, visited,visualGraph,pos,obstruccion,affectedNode), interval=400,frames=len(order),repeat=False)
    FFwriter = animation.FFMpegWriter()
    anim.save('../star'+str(rd.randint(1,50))+'.mp4', writer=FFwriter,dpi=300)
    plt.close()

#ALGORITHM FUNCTIONS

def randomEvent (graph) :
    adj_list = graph.adjacency_list
    
    nodes = list(adj_list.keys())
    randomNode = rd.choice(nodes)
    obstruccion = rd.randint(50,100)
    randomEdge = rd.randint(0, len(graph.adjacency_list[randomNode]) - 1)
    

    edgeNode,value = graph.adjacency_list[randomNode][randomEdge]
    print('NODO AFECTADO Y ARCO AFECTADO - '+randomNode,edgeNode)
    graph.adjacency_list[randomNode][randomEdge] = (edgeNode,obstruccion)
    for edge in graph.adjacency_list[edgeNode]:
        if (edge[0] == randomNode):
            graph.adjacency_list[edgeNode][graph.adjacency_list[edgeNode].index(edge)] = (randomNode,obstruccion)
    return [randomNode,edgeNode]


def realTime (graph,start_node,final_node) :
    order = graph.a_star_algorithm(start_node,final_node)
    previousOrder = []
    animateGraph(graph,order)
    affectedNodes = randomEvent(graph)
    print('ANTIGUO ORDEN = '+' '.join(order))
    print('NODO Y ARCO AFECTADO = '+affectedNodes[0]+ ' - ' + affectedNodes[1])
    for i in range(len(order)):
        if i < len(order)-1 :
            if (order[i+1] == affectedNodes[0] and affectedNodes[1] in order) or (order[i] == affectedNodes[0] and affectedNodes[1] in order):
                previousOrder = order
                order = graph.a_star_algorithm(order[i],final_node)
                print('NUEVO ORDEN'+' = '+' '.join(previousOrder[0:i]) + ' ' +  ' '.join(order))
    if len(previousOrder) == 0: 
        print('NO HUBO CAMBIOS EN LA RUTA')

def fastestContinuousRoute(graph,start_node,final_node) :
    continuousRoute = []
    here = start_node
    while (here != final_node) :
        route = graph.a_star_algorithm(here,final_node)
        isNewSample = False
        while (isNewSample == False and here != final_node):
            continuousRoute.append(here)
            next = getNext(here,route)
            weight = getPathWeight(graph,here,next)
            if rd.randint(1,3) == 2:
                affectedNodes = randomEvent(graph)
                if affectedNodes[0] in route and affectedNodes[1] in route:
                    isNewSample = True  
                    here = next
                    break
            here = next
        if here == final_node:
            continuousRoute.append(here)
    return continuousRoute

def getNext(current,route):
    if (current == route[len(route)-1]):
        return current
    else:
        return route[route.index(current)+1]

def getPathWeight(graph,start_node,final_node):
    adjacency_list = graph.adjacency_list
    for node,edges in adjacency_list.items():
        for edge in edges: 
            if node == start_node and edge[0] == final_node:
                return adjacency_list[node][adjacency_list[node].index(edge)][1]

# VARIABLES

adjac_list = {
    'A': [('B', 10), ('C', 12), ('D', 5)],
    'B': [('A', 10), ('E',11)],
    'C': [('A', 12), ('D', 6),('E', 11),('F',8)],
    'D': [('A',5), ('C',6),('F',14)],
    'E': [('B',11), ('C',11)],
    'F': [('C',8), ('D',14)]
}

graph = Graph(adjac_list)
start_node = 'B'
final_node = 'F'

print(fastestContinuousRoute(graph,start_node,final_node))

# realTime(graph,start_node,final_node)

# time.sleep(5)
# animateGraph(graph,start_node,final_node)