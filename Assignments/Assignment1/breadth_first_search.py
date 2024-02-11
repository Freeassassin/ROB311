from collections import deque
import numpy as np
from search_problems import Node, GraphSearchProblem
import networkx as nx
import matplotlib.pyplot as plt
def breadth_first_search(problem):
    """
    Implement a simple breadth-first search algorithm that takes instances of SimpleSearchProblem (or its derived
    classes) and provides a valid and optimal path from the initial state to the goal state. Useful for testing your
    bidirectional and A* search algorithms.

    :param problem: instance of SimpleSearchProblem
    :return: path: a list of states (ints) describing the path from problem.init_state to problem.goal_state[0]
             num_nodes_expanded: number of nodes expanded by the search
             max_frontier_size: maximum frontier size during search
    """
    max_frontier_size = 0
    
    frontier = deque([Node(None,problem.init_state,None,0)])  # Initialize the frontier with the initial state
    explored = set()  # Initialize an empty set to keep track of explored nodes
    while len(frontier):
        if len(frontier) > max_frontier_size:
            max_frontier_size = len(frontier)
        node = frontier.pop()  # Choose the deepest node in the frontier
        explored.add(node.state)
        if problem.goal_test(node.state):  # Check if the node contains a goal state
            # Return the solution path
            return problem.trace_path(node), len(explored), max_frontier_size
        for action in problem.get_actions(node.state):
            child = problem.get_child_node(node,action)
            
            if child.state not in explored and child not in frontier:
                explored.add(child.state)
                if problem.goal_test(child.state):
                    return problem.trace_path(child), len(explored), max_frontier_size
                frontier.append(child)
    return None, len(explored), max_frontier_size


if __name__ == '__main__':
    # Simple example
    goal_states = [0]
    init_state = 9
    V = np.arange(0, 10)
    E = np.array([[0, 1],
                  [1, 2],
                  [2, 3],
                  [3, 4],
                  [4, 5],
                  [5, 6],
                  [6, 7],
                  [7, 8],
                  [8, 9],
                  [0, 6],
                  [1, 7],
                  [2, 5],
                  [9, 4]])
    # Create a NetworkX graph object
    G = nx.Graph()

    # Add nodes from the vertex array
    G.add_nodes_from(V)

    # Add edges from the edge array
    for edge in E:
        G.add_edge(*edge)

    # Choose a layout for better visualization
    layout = nx.spring_layout(G)  # You can try other layouts too

    # Draw the graph 
    nx.draw(G, pos=layout, with_labels=True, node_color='skyblue')

    # Show the plot
    plt.show()
    problem = GraphSearchProblem(goal_states, init_state, V, E)
    path, num_nodes_expanded, max_frontier_size = breadth_first_search(problem)
    correct = problem.check_graph_solution(path)
    print("Solution is correct: {:}".format(correct))
    print(path)

    # Use stanford_large_network_facebook_combined.txt to make your own test instances
    E = np.loadtxt('stanford_large_network_facebook_combined.txt', dtype=int)
    V = np.unique(E)
    goal_states = [349]
    init_state = 0
    problem = GraphSearchProblem(goal_states, init_state, V, E)
    path, num_nodes_expanded, max_frontier_size = breadth_first_search(problem)
    correct = problem.check_graph_solution(path)
    print("Solution is correct: {:}".format(correct))
    print(path)