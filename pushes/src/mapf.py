#!/usr/bin/env python3

"""
Copyright 2021, University of Freiburg
Bachelorarbeit - Foundations of Artificial Intelligence

Author: Marco Kaiser <kaiserm@informatik.uni-freiburg.de>

Description:

Usage:

"""

import time

import src.a_star as a_star
import src.classes as classes

def pp(env):
    
    print("Checking if the starting positions are valid...")
    
    
    # Check if the positions are valid and otherwise try to move alle agents one forward
    for iters in range(5):

        valid = True
        positions = []
        for handle in range(env.get_num_agents()):

            agent = env.agents[handle]

            if agent.position not in positions and agent.position is not None:
                positions.append(agent.position) 
            else:
                valid = False
                print(f"The positions for the {iters} Step is not valid. Try one more step...")
                env.step({i : 2 for i in range(env.get_num_agents())})
                break
                
        if valid:
            print("The positions are valid! Start searching...\n")
            break
            
    if not valid:
        print("Coudln't find valid starting positions. Try other enviroment!\n")
        return -1
            
    
    schedules = {}
    seen = {}
    
    for handle in range(env.get_num_agents()):
        
        agent = env.agents[handle]
        
        position = agent.position
        direction = agent.direction
        target = agent.target
        
        print(f"Agent: {handle:2} starting from: {position} - heading to: {target}", end=" --> ")

        
        time_start = time.time()
        
        # Get the path of the current agent with the already seen states
        res = a_star.search(handle, position, direction, target, seen, env)
        
        print(f"in {time.time()-time_start:5f}sec")
        
        for node in res.nodes:
            seen.setdefault(node.timestamp, []).append(node)

        if res.status == classes.A_Star_result.fail:
            print(f"Agent <{handle}> couldn't find a valid schedule!")
            return -1

        schedules[handle] = res
        
    return schedules


if __name__ == '__main__':
    pass
