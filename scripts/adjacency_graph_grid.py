#! /usr/bin/env python

from copy import deepcopy
from typing import List

class Node:
    def __init__(self, ID) -> None:
        self.adjacent = []
        self.ID = ID

    def add_adjacent(self, node) -> None:
        self.adjacent.append(node)
    
    def get_adjacent(self):
        return self.adjacent

    
    def get_ID(self) -> int:
        return self.ID

class Graph:
    def __init__(self) -> None:
        self.nodes = []

    def add_node(self, node) -> None:
        self.nodes.append(node)

    def get_node_by_ID(self, id) -> Node:
        for n in self.nodes:
            if n.get_ID() == id:
                return n


def generate_graph() -> Graph:
    g = Graph()

    grid_1 = Node(1)
    grid_2 = Node(2)
    grid_3 = Node(3)
    grid_4 = Node(4)
    grid_5 = Node(5)
    grid_6 = Node(6)
    grid_7 = Node(7)
    grid_8 = Node(8)
    grid_9 = Node(9)

    grid_1.add_adjacent(grid_2)
    grid_1.add_adjacent(grid_4)

    grid_2.add_adjacent(grid_5)
    grid_2.add_adjacent(grid_7)
    grid_2.add_adjacent(grid_1)

    grid_3.add_adjacent(grid_4)

    grid_4.add_adjacent(grid_3)
    grid_4.add_adjacent(grid_1)

    grid_5.add_adjacent(grid_6)
    grid_5.add_adjacent(grid_7)

    grid_6.add_adjacent(grid_5)
    grid_6.add_adjacent(grid_9)

    grid_7.add_adjacent(grid_8)
    grid_7.add_adjacent(grid_2)

    grid_8.add_adjacent(grid_7)

    grid_9.add_adjacent(grid_6)

    g.add_node(grid_1)
    g.add_node(grid_2)
    g.add_node(grid_3)
    g.add_node(grid_4)
    g.add_node(grid_5)
    g.add_node(grid_6)
    g.add_node(grid_7)
    g.add_node(grid_8)
    g.add_node(grid_9)

    return g

def is_complete(path_of_nodes) -> bool:

    grid = {
        1 : False,
        2 : False,
        3 : False,
        4 : False,
        5 : False,
        6 : False,
        7 : False,
        8 : False,
        9 : False
    }

    for node in path_of_nodes:
        grid[node.get_ID()] = True

    complete = True

    for i, _ in enumerate(grid):
        if grid[i+1] == False:
            complete = False
        
    return complete

def bfs_tsp(graph) -> List[Node]:
    starting_node = graph.get_node_by_ID(1)
    starting_point = [starting_node, [starting_node]]

    open_set = []

    open_set.append(starting_point)

    while len(open_set) != 0:
        node, path = open_set.pop(0)

        print(node.get_ID())
        print(len(path))

        adjacent_nodes = node.get_adjacent()
        for adj in adjacent_nodes:
            new_path = deepcopy(path)
            new_path.append(adj)
            open_set.append([adj, new_path])

        if(is_complete(path)):
            return path
    
    print("failed to find path")




if __name__ == "__main__":
    adjacency_graph = generate_graph()
    shortest_tour = bfs_tsp(adjacency_graph)

    i = 0
    for node in shortest_tour:
        print(i, node.get_ID())
        i += 1
    
