#pragma once

#include <a_star.hpp>

template <typename FlatlandCBS, typename State, typename GridLocation, typename Agent, typename Action>
class SearchCBS {
  public:
    SearchCBS(FlatlandCBS& flatlandCBS) : m_flatlandCBS(flatlandCBS) {}

    bool search(std::vector<PlanResult<Action, State> >& solution) {
      

      // Initialize the first HighLevelNode
      HighLevelNode start;
      start.solution.resize(m_flatlandCBS.agents.size());
      start.constraints.resize(m_flatlandCBS.agents.size());
      start.cost = 0;
      start.id = 0;
    
      for (int handle = 0; (unsigned) handle < m_flatlandCBS.agents.size(); handle++) {
        Agent a = m_flatlandCBS.agents[handle];
        std::cout << a << std::endl;

        AStar_t astar(m_flatlandCBS, &start.constraints[handle]);
        astar.search(a, start.solution[handle]);
      }

      std::cout << start << std::endl;

      solution = start.solution;

      return true;
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
