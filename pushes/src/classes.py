#!/usr/bin/env python3

"""
Copyright 2021, University of Freiburg
Bachelorarbeit - Foundations of Artificial Intelligence

Author: Marco Kaiser <kaiserm@informatik.uni-freiburg.de>

Description:

Usage:

"""

direction_to_str = {-1 : "pass...", 0: "North", 1: "East", 2: "South", 3: "West"}
action_to_str = {-1 : "pass...", 0 : "Do nothing", 1: "Left", 2 : "Forward", 3 : "Right", 4 : "Stop moving"}


class NextCell():
    
    def __init__(self, position, action, cost, direction):
        self.position = position
        self.action = action
        self.cost = cost
        self.direction = direction

    def __repr__(self):
        return f"{self.position} | {action_to_str[self.action]} | {self.cost} | {direction_to_str[self.direction]}"


class Node:
    def __init__(self, position, parent=None, action=None, cost=-1, direction=None, timestamp=-1):
        self.position = position
        self.parent = parent
        self.action = action
        self.cost = cost
        self.direction = direction
        self.timestamp = timestamp
        
        self.state = (position, direction, timestamp)
    
    def __lt__(self, other):
        return self.cost < other.cost
    
    def __repr__(self):
        if self.action is not None and self.direction is not None:
            return f"{self.position} | {action_to_str[self.action]:7s} | {self.cost:3} | {direction_to_str[self.direction]:5s} | t={self.timestamp}"
        elif self.action:
            return f"{self.position} | {action_to_str[self.action]:7s} | t={self.timestamp}"
        else:
            return f"{self.position} | t={self.timestamp}"
    
    def check_possible(self, other_obstacles):
        # Get all the obstacles in the current timestamp
        res = other_obstacles.get(self.timestamp)
        
        # When there is no obstacle with the current timestamp, then there is no conflict
        if res is None: return True

        # Check if there is a obstacle with the same position. If there is an overlap, otherwise it is ok
        for n in res:
            
            if self.position == n.position: return False
            
        return True
            
    @staticmethod
    def make_root_node(postion, direction):
        return Node(postion, None, None, 0, direction, 0)

    @staticmethod
    def make_node(position, parent, action, cost, direction):
        return Node(position, parent, action, cost, direction, parent.timestamp + 1)

    @staticmethod
    def extract_solution(node):
        sol = []
        
        while node.parent is not None:
            sol.append(Node(node.position, action=node.action, timestamp=node.timestamp))

            node = node.parent
        
        return sol[::-1]


class A_Star_result():
    succes = 1
    fail = 2
    
    def __init__(self, status=succes, nodes=[], iterations=-1):
        self.status = status
        self.nodes = nodes
        
        self.actions = []
        self.positions = []

        for node in nodes:
            self.actions.append(node.action)
            self.positions.append(node.position)
            
        self.iterations = iterations

    def __repr__(self):
        if self.status == A_Star_result.succes:
            return f"Succes! actions: {self.actions}, positions: {self.positions}"
        else:
            return f"No Solution! Failed in {self.iterations} iterations..."
        
    def is_empty(self):
        return len(self.actions) == 0
    
    def get_n_action(self, n):
        if n < 0 or n >= len(self.actions):
            raise "Error: Trying to get acces to an action with an undifinded timestep!"
            
        return self.actions[n]
    
    def pop(self, n=0):
        if n < 0 or n >= len(self.actions):
            raise "Error: Trying to get acces to an action with an undifinded timestep!"
        
        return (self.actions.pop(n), self.positions.pop(n))


if __name__ == '__main__':
    pass