#!/usr/bin/env python3

"""
Copyright 2021, University of Freiburg
Bachelorarbeit - Foundations of Artificial Intelligence

Author: Marco Kaiser <kaiserm@informatik.uni-freiburg.de>

Description:

Usage:

"""

import heapq
import src.next_cell as nc
import src.classes as classes



def search(handle, position, direction, target, other_obstacles, env):

    open_list = []
    heapq.heappush(open_list, classes.Node.make_root_node(position, direction))
    
    closed_list = []
    distance = {}

    i = 0
    while len(open_list) > 0:
        
        node = heapq.heappop(open_list)

        if node.state not in closed_list or node.cost < distance[node.state]:
            
            closed_list.append(node.state)
            distance[node.state] = node.cost
            
            # Return the calculated nodes when the target is reached
            if node.position == target:
                return classes.A_Star_result(classes.A_Star_result.succes, classes.Node.extract_solution(node))
        
            # List with all the subsequent cells created as nodes
            next_cells = nc.calc_next_cell(node.position, node.direction, handle, env)
            succ_cells = [classes.Node.make_node(n.position, node, n.action, n.cost, n.direction) for n in next_cells]
            
            # Check if the subsequent cells has an overlap with other obstacles to the specific timestamp
            app_cells = [pos_node.check_possible(other_obstacles) for pos_node in succ_cells]
    
    
            # Zusammenfassung: Nimm den Zeitpunkt des Stopp_moving Fehlers und lösche die Nodes, welche
            # den gleichen Zeitpunkt und die gleiche Position wie andere obsacles haben
            if app_cells[0] == False:
                
                pass
                
            
            # When there is a False in the rest of the applicable cells, without the first stop_moving one
            elif not all(app_cells[1:]):
                
                # Die Stopp-Aktion wird immer hinzugefügt
                heapq.heappush(open_list, succ_cells[0])

                # Alle Aktionen, die False sind werden rausgeworfen
                for n, app in enumerate(app_cells[1:]):

                    if not app: continue
                    
                    heapq.heappush(open_list, succ_cells[n+1])

            else:
                
                # If there is no stop_moving or other error, then all the next cells can get appended
                for succ in succ_cells:
                    
                    heapq.heappush(open_list, succ)

        i += 1

    return classes.A_Star_result(classes.A_Star_result.fail, iterations=i)


if __name__ == '__main__':
    pass