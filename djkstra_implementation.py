import sys
import time
class Vertex:
    def __init__(self, node):
        self.id = node
        self.adjacent = {}
        # Set distance to infinity for all nodes
        self.distance = sys.maxsize - 10000
        # Mark all nodes unvisited        
        self.visited = False  
        # Predecessor
        self.previous = None

    def add_neighbor(self, neighbor, weight=0):
        self.adjacent[neighbor] = weight

    def get_connections(self):
        return self.adjacent.keys()  

    def get_id(self):
        return self.id

    def get_weight(self, neighbor):
        return self.adjacent[neighbor]

    def set_weight_infinite(self, neighbor, value):
        self.adjacent[neighbor] = value

    def set_distance(self, dist):
        self.distance = dist

    def get_distance(self):
        return self.distance

    def set_previous(self, prev):
        self.previous = prev

    def set_visited(self):
        self.visited = True

    def __str__(self):
        return str(self.id)# + ' adjacent: ' + str([x.id for x in self.adjacent])

    def __lt__(self, other):
        return self.distance < other.distance

class Graph:
    def __init__(self):
        self.vert_dict = {}
        self.num_vertices = 0

    def __iter__(self):
        return iter(self.vert_dict.values())

    def add_vertex(self, node):
        self.num_vertices = self.num_vertices + 1
        new_vertex = Vertex(node)
        self.vert_dict[node] = new_vertex
        return new_vertex

    def get_vertex(self, n):
        if n in self.vert_dict:
            return self.vert_dict[n]
        else:
            return None

    def add_edge(self, frm, to, cost = 0):
        if frm not in self.vert_dict:
            self.add_vertex(frm)
        if to not in self.vert_dict:
            self.add_vertex(to)

        self.vert_dict[frm].add_neighbor(self.vert_dict[to], cost)
        self.vert_dict[to].add_neighbor(self.vert_dict[frm], cost)

    def get_vertices(self):
        return self.vert_dict.keys()

    def set_previous(self, current):
        self.previous = current

    def get_previous(self):
        return self.previous
    
    def reset(self):
        for vertex in self.vert_dict:
            # Mark all nodes unvisited        
            self.vert_dict[vertex].visited = False  
            # Predecessor
            self.vert_dict[vertex].previous = None
            # Set infinite distance
            self.vert_dict[vertex].distance = sys.maxsize - 10000
            

    def printGraph(self):
        for v in self.get_vertices():
            for w in self.vert_dict[v].get_connections():
                print("( %s , %s, %3d)"  % (v, w, self.get_vertex(v).get_weight(w)))
            

import random as rd

def randomBlockNode(aGraph):
    adj_list = aGraph.vert_dict
    vertexes = list(adj_list.keys())
    randomVertex = rd.choice(vertexes)
    randomArc = rd.choice(list(adj_list[randomVertex].get_connections()))
    adj_list[randomVertex].set_weight_infinite(randomArc, sys.maxsize)
    print("NODO con ARCO BLOCKED -> ", randomVertex, " X X X ", randomArc.get_id())
    for vertex in adj_list:
        if(vertex == randomArc.get_id()):
            adj_list[randomArc.get_id()].set_weight_infinite(aGraph.get_vertex(randomVertex), sys.maxsize - 10000)

    return randomVertex, randomArc.get_id()
def shortest_path(graph, start, target, path):
    ''' make shortest path from v.previous'''
    dijkstra(graph, start)

    if target.previous:
        path.append(target.previous.get_id())
        shortest_path(graph, start, target.previous, path)    
    return path[::-1]

def shortest(graph, start, target, path):
    ''' make shortest path from v.previous'''
    path = shortest_path(graph,start,target,path)
    print("PATH ORIGINAL - ", path)
    while True:
        if path:
            element = path.pop(0)
            element_node = graph.get_vertex(element)
        else:
            break
        print("RECORRIDO: ", element, " *** RESTANTE: ", path)
        is_path_blocked = True if rd.randint(1,3)==2 else False
        if is_path_blocked and path:
            nodo_b1, nodo_b2 = randomBlockNode(graph)
            if nodo_b1 in path and nodo_b2 in path:
                graph.reset()
                print("BLOQUEO DETECTADO... PLANEANDO RE-RUTA *X*")
                path = shortest_path(graph, graph.get_vertex(path[0]), target, path)
        if element_node == target:
            print("TERMINADO")
            break    
    return

import heapq

def dijkstra(aGraph, start):
    # Set the distance for the start node to zero 
    start.set_distance(0)

    # Put tuple pair into the priority queue
    unvisited_queue = [(v.get_distance(),v) for v in aGraph]
    heapq.heapify(unvisited_queue)

    while len(unvisited_queue):
        # Pops a vertex with the smallest distance 
        uv = heapq.heappop(unvisited_queue)
        current = uv[1]
        current.set_visited()

        #for next in v.adjacent:
        for next in current.adjacent:
            # if visited, skip
            if next.visited:
                continue
            new_dist = current.get_distance() + current.get_weight(next)
            if new_dist < next.get_distance():
                next.set_distance(new_dist)
                next.set_previous(current)

        # Rebuild heap
        # 1. Pop every item
        while len(unvisited_queue):
            heapq.heappop(unvisited_queue)
        # 2. Put all vertices not visited into the queue
        unvisited_queue = [(v.get_distance(),v) for v in aGraph if not v.visited]
        heapq.heapify(unvisited_queue)


if __name__ == '__main__':

    g = Graph()

    g.add_vertex('a')
    g.add_vertex('b')
    g.add_vertex('c')
    g.add_vertex('d')
    g.add_vertex('e')
    g.add_vertex('f')

    g.add_edge('a', 'b', 10)  
    g.add_edge('a', 'c', 12)
    g.add_edge('a', 'd', 5)
    g.add_edge('b', 'a', 10)
    g.add_edge('b', 'e', 11)
    g.add_edge('c', 'a', 12)
    g.add_edge('c', 'd', 6)
    g.add_edge('c', 'e', 11)
    g.add_edge('c', 'f', 8)
    g.add_edge('d', 'a', 5)
    g.add_edge('d', 'c', 6)
    g.add_edge('d', 'f', 14)
    g.add_edge('e', 'b', 11)
    g.add_edge('e', 'c', 11)
    g.add_edge('f', 'c', 8)
    g.add_edge('f', 'd', 14)    
    
    print("Graph data:")

    
    
    start_time=time.perf_counter()  
    start = g.get_vertex('b')      
    dijkstra(g, start) 
    end_time=time.perf_counter()  
    target = g.get_vertex('f')
    print("SHORTEST PATH FROM B TO F")
    path = [target.get_id()]
    shortest(g, start, target, path)
    #print("The shortest path : %s" %(path[::-1]))
    print("CALCULATED TIME (seconds): ", end_time-start_time)