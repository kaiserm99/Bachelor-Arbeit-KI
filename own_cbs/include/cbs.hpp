/* Copyright 2021, University of Freiburg
 * Bachelorarbeit - Foundations of Artificial Intelligence
 *
 * Author: Marco Kaiser <kaiserm@informatik.uni-freiburg.de>
 *
 * Description: 
 *
 * Usage: 
 */

#pragma once

#include <a_star.hpp>

template <typename FlatlandCBS, typename State, typename GridLocation, typename Agent, typename Action>
class SearchCBS {
  public:
    SearchCBS(FlatlandCBS& flatlandCBS) : m_flatlandCBS(flatlandCBS) {}


    bool search(std::vector<PlanResult<Action, State> >& solution) {

      // Initialize the first HighLevelNode
      HighLevelNode start;
      start.solution.resize(m_flatlandCBS.m_agents.size());
      start.constraints.resize(m_flatlandCBS.m_agents.size());
      start.cost = 0;
      start.id = 0;
    
      /*
      Constraints tst; 
      // tst.add(Constraint(1, 7, 18, 19));
      tst.addInitial(10); 
      // tst.add(Constraint(13, 20, 17, 23));
      
      start.constraints[0] = tst;
      */


      // Compute the initial search and check if every solution is somehow possigle
      for (int handle = 0; (unsigned) handle < m_flatlandCBS.m_agents.size(); handle++) {
        Agent a = m_flatlandCBS.m_agents[handle];
        std::cout << a << std::endl;

        AStar_t astar(m_flatlandCBS, &start.constraints[handle]);

        // If the search fails, there is no way that an agent can reach its target
        if (!astar.search(a, start.solution[handle])) return false;

        start.cost += start.solution[handle].cost;
      }

      typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>, boost::heap::mutable_<true>> open;
      auto h = open.push(start);
      (*h).handle = h;

      solution.clear();
      int id = 1;

      // Only for infinite loops
      size_t currentConstraint = -1;
      std::pair<size_t, size_t> currentDoubleConstraint = std::make_pair(-1, -1);
      size_t count = 0;
      
      while (!open.empty()) {

        if (count == 10000) return false;

        HighLevelNode P = open.top();  // Get the high level node with the lowest costs

        m_flatlandCBS.onExpandHighLevelNode();  // Statistics

        open.pop();

        std::vector<std::pair<size_t, Constraints>> resultConstraints;
        std::vector<std::pair< std::pair<size_t, size_t> , std::pair<Constraints, Constraint>>> resultDoubleConstraints;

        if (!m_flatlandCBS.getFirstConflict(P.solution, P.constraints, resultConstraints, resultDoubleConstraints)) {

          std::cout << "Final HighLevelNode:" << std::endl << P << std::endl << std::endl;

          std::cout << "done; cost: " << P.cost << std::endl;
          solution = P.solution;

          return true;
        }


        for (const auto& c : resultDoubleConstraints) {

          size_t handle1 = c.first.first;
          size_t handle2 = c.first.second;

          HighLevelNode newNode = P;
          newNode.id = id;
          newNode.cost -= P.solution[handle1].cost;
          newNode.cost -= P.solution[handle2].cost;

          newNode.constraints[handle1].extend(c.second.first);
          newNode.constraints[handle2].add(c.second.second);

          
          // if (DEBUG) std::cout << "Constrained " << handle1 << ", " << c.second.first << std::endl;
          // if (DEBUG) std::cout << "Constrained " << handle2 << ", " << c.second.second << std::endl;

          if (handle1 == currentDoubleConstraint.first && handle2 == currentDoubleConstraint.second) count++;
          else {
            currentDoubleConstraint = std::make_pair(handle1, handle2);
            count = 0;
          }

          // std::cout << handle1 << "," << handle2 << "|";

          AStar_t astar(m_flatlandCBS, &newNode.constraints[handle1]);
          Agent a = m_flatlandCBS.m_agents[handle1];
    
          if (!astar.search(a, newNode.solution[handle1])) continue;


          AStar_t astarTwo(m_flatlandCBS, &newNode.constraints[handle2]);
          a = m_flatlandCBS.m_agents[handle2];
          if (!astarTwo.search(a, newNode.solution[handle2])) continue;

          newNode.cost += newNode.solution[handle1].cost;
          newNode.cost += newNode.solution[handle2].cost;
          // std::cout << newNode << std::endl;
          auto h = open.push(newNode);
          (*h).handle = h;

          ++id;
        }

      
        for (const auto& c : resultConstraints) {

          size_t handle = c.first;

          HighLevelNode newNode = P;
          newNode.id = id;
          newNode.cost -= P.solution[handle].cost;

          newNode.constraints[handle].extend(c.second);

          if (DEBUG) std::cout << "Constrained " << handle << ", " << c.second << std::endl;
          
          // std::cout << handle << "|";

          if (handle == currentConstraint) count++;
          else {
            currentConstraint = handle; 
            count = 0;
          } 

          AStar_t astar(m_flatlandCBS, &newNode.constraints[handle]);
          Agent a = m_flatlandCBS.m_agents[handle];
    
          if (astar.search(a, newNode.solution[handle])) {

            newNode.cost += newNode.solution[handle].cost;
            // std::cout << newNode << std::endl;
            auto h = open.push(newNode);
            (*h).handle = h;
          }

          ++id;
        }
      }

      return false;
    }

  private:
    struct HighLevelNode {
      std::vector<PlanResult <Action, State> > solution;
      std::vector<Constraints> constraints;

      int id;
      int cost;

      // TODO: Look if good
      bool operator<(const HighLevelNode& n) const {
        if (cost != n.cost) return cost > n.cost;
        else return id > n.id;
      }

      friend std::ostream& operator<<(std::ostream& os, const HighLevelNode& c) {
        os << "id: " << c.id << " cost: " << c.cost << std::endl;
        for (size_t handle = 0; handle < c.solution.size(); handle++) {
          os << "Agent: " << handle << std::endl;
          os << " States:" << std::endl;
          for (const auto& s : c.solution[handle].states) {
            os << "  " << s << std::endl;
          }
          os << " Constraints:" << std::endl;
          os << c.constraints[handle] << std::endl;
          os << " cost: " << c.solution[handle].cost << std::endl;
        }
        return os;
      }


      typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>, boost::heap::mutable_<true> >::handle_type handle;
    };
  
  
  private:
    FlatlandCBS& m_flatlandCBS;

    typedef AStar<FlatlandCBS, State, Agent, Action> AStar_t;
};
