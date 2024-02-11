from collections import deque
import numpy as np
from search_problems import Node, GraphSearchProblem

def bidirectional_search(problem):
    """
        Implement a bidirectional search algorithm that takes instances of SimpleSearchProblem (or its derived
        classes) and provides a valid and optimal path from the initial state to the goal state.

        :param problem: instance of SimpleSearchProblem
        :return: path: a list of states (ints) describing the path from problem.init_state to problem.goal_state[0]
                 num_nodes_expanded: number of nodes expanded by the search
                 max_frontier_size: maximum frontier size during search
        """
    max_frontier_size = 0
    num_nodes_expanded = 0
    
    forward_frontier = deque([Node(None,problem.init_state,None,0)])  # Initialize the frontier with the initial state
    backward_frontier = deque([Node(None,problem.goal_states[0],None,0)])  # Initialize the frontier with the initial state
    forward_explored = set()  # Initialize an empty set to keep track of explored nodes
    backward_explored = set()  # Initialize an empty set to keep track of explored nodes
    
    front_discard = dict()
    back_discard = dict()
    while len(forward_frontier) + len(backward_frontier):
        max_frontier_size = max(max_frontier_size,len(forward_frontier)+len(backward_frontier)) 
                
        for _ in range(len(forward_frontier)):
            
            forward_node = forward_frontier.popleft()  # Choose the deepest node in the frontier
        
            forward_explored.add(forward_node.state)
            front_discard[forward_node.state] = forward_node 
        
            num_nodes_expanded +=1  
            
            actions = problem.get_actions(forward_node.state)
        
            for action in actions:
                
                if action[1] not in forward_explored:
                    
                    child = problem.get_child_node(forward_node,action)
                    
                    forward_frontier.append(child)
                    
                    if (child.state in backward_explored):
                        
                        backward = back_discard[child.state]
                        
                        path = problem.trace_path(child)[:-1]
                        path.extend(problem.trace_path(backward,problem.goal_states[0])[::-1])
                        
                        return path, num_nodes_expanded, max_frontier_size
        back_discard.clear()
        
        for _ in range(len(backward_frontier)):
            
            backward_node = backward_frontier.popleft()  # Choose the deepest node in the frontier
        
            backward_explored.add(backward_node.state)
            back_discard[backward_node.state] = backward_node 
        
            num_nodes_expanded +=1  
            
            actions = problem.get_actions(backward_node.state)
        
            for action in actions:
                
                if action[1] not in backward_explored:
                    
                    child = problem.get_child_node(backward_node,action)
                    
                    backward_frontier.append(child)
                    
                    if (child.state in forward_explored):
                        
                        forward = front_discard[child.state]
                        
                       
                        path = problem.trace_path(forward)[:-1]
                        path.extend(problem.trace_path(child,problem.goal_states[0])[::-1])
                        
                        
                        return path, num_nodes_expanded, max_frontier_size
        
        
        front_discard.clear()
        
    return [], num_nodes_expanded, max_frontier_size