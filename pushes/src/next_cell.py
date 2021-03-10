#!/usr/bin/env python3

"""
Copyright 2021, University of Freiburg
Bachelorarbeit - Foundations of Artificial Intelligence

Author: Marco Kaiser <kaiserm@informatik.uni-freiburg.de>

Description:

Usage:

"""

from flatland.core.grid.grid4_utils import get_new_position
import src.classes as classes

def calc_next_cell(position, direction, handle, env):
    
    possible_transitions = env.rail.get_transitions(*position, direction)

    # This should be never true when searching with the a star algorithm
    if all(d == 0 for d in possible_transitions):
        print("This Direction is not permissable!")
        return -1
    
    # Initalize the next_cells with the action when there is done nothing/waiting (4), because this is alway possible
    next_cells = [classes.NextCell(position, 4, env.distance_map.get()[handle, position[0], position[1], direction], direction)]

    # Loop trough all the possible dirrections the agent can reach from current direction
    for d in [(direction + i) % 4 for i in range(-1, 2)]:
        
        if possible_transitions[d]:
            
            # Die neue Position, wenn man die jeweilige direction 
            new_position = get_new_position(position, d)
            
            
            # Die Distanz von einer Position zum Ziel des jeweiligen Agenten
            dist = env.distance_map.get()[handle, new_position[0], new_position[1], d]
            
            # Check the given directions and map it to the corresponding action
            if d == direction:
                next_cells.append(classes.NextCell(new_position, 2, dist, d))

            elif (d + 1) % 4 == direction:
                next_cells.append(classes.NextCell(new_position, 1, dist, d))
                
            elif (d - 1) % 4 == direction:
                next_cells.append(classes.NextCell(new_position, 3, dist, d))
    
    else:
    
        # Check if the transition is an dead End
        if possible_transitions[(direction + 2) % 4] == 1:
            direction = (direction + 2) % 4

            new_position = get_new_position(position, direction)

            dist = env.distance_map.get()[handle, new_position[0], new_position[1], direction]

            next_cells.append(classes.NextCell(new_position, 2, dist, direction))

    return next_cells