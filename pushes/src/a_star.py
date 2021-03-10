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


def delete_open_list(t, open_list, other_obstacles, handle=-1):    
        
    for n, timestamp in enumerate(range(t, 2*t)):
        
        res = other_obstacles.get(timestamp)
        
        if res is None:
            return open_list
            
        # Wenn der Zeitpunkt und die Position gelich wie eine der obstacles ist, dann lösche dies aus der open_list
        for obs in res:        
            
            open_list = [ op for op in open_list if not (op.timestamp == timestamp - (2 * n + 1) and op.position == obs.position)]
            
    return open_list


def delete_open_list_gerade(t, open_list, other_obstacles, handle=-1):
     
    for n, timestamp in enumerate(range(t+1, 2*t)):
        
        res = other_obstacles.get(timestamp)

        if res is None: return open_list
            
        for obs in res:
            
            open_list = [ op for op in open_list if not (op.timestamp == timestamp - (2 * (n+1)) and op.position == obs.position)]
            
    return open_list



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
                
                t = succ_cells[0].timestamp
                
                # Check if the other actions are applicable. 
                for n, app in enumerate(app_cells[1:]):

                    if not app: continue
                    
                    pos_succ = succ_cells[n+1]
                    
                    # If the position is not the same as in the previous timestamp, then it is applicaple
                    insert = True
                    for obs in other_obstacles[t-1]:
                    
                        if pos_succ.position == obs.position:
                            insert = False
                            break
                    
                    if insert: heapq.heappush(open_list, pos_succ)

                # Always delete all the state with the same position an coressponding timestamp
                open_list = delete_open_list(t, open_list, other_obstacles, handle)
                
            
            # When there is a False in the rest of the applicable cells, without the first stop_moving one
            elif not all(app_cells[1:]):
                
                t = succ_cells[1].timestamp
                
                # Im folgenden wird geprüft, ob die stop_moving action mit dem jeweiligen Zeitstamp
                # auch anwendbar ist. Wenn zwei Züge sich zum Zeitpunkt X treffen, so muss überprüft werden,
                # ob an im X+1-ten obstacle die gleiche Position wie in X vorherrscht
                pos_succ = succ_cells[0]
                
                insert = True
                for obs in other_obstacles[t+1]:
                    
                    if pos_succ.position == obs.position:
                        insert = False
                        break
                
                if insert: heapq.heappush(open_list, pos_succ)

                # Check if the other actions are applicable. The are if app == True. Then it can get appended to the
                # open list for further preparation
                for n, app in enumerate(app_cells[1:]):

                    if not app: continue
                    
                    heapq.heappush(open_list, succ_cells[n+1])

                # Always delete all the state with the same position an coressponding timestamp
                open_list = delete_open_list_gerade(t, open_list, other_obstacles, handle)

            else:
                
                # If there is no stop_moving or other error, then all the next cells can get appended
                for succ in succ_cells:
                    
                    heapq.heappush(open_list, succ)

        i += 1

    return classes.A_Star_result(classes.A_Star_result.fail, iterations=i)


if __name__ == '__main__':
    pass