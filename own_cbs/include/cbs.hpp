#pragma once

#include <a_star.hpp>

template <typename FlatlandCBS, typename State, typename GridLocation, typename Agent, typename Action>
class SearchCBS {
  public:
    SearchCBS(FlatlandCBS& flatlandCBS) : m_flatlandCBS(flatlandCBS) {}

    bool search() {
      AStar_t astar(m_flatlandCBS);



      for (auto& a : m_flatlandCBS.agents) {
        std::cout << a << std::endl;
        
        astar.search(a);

        for (const auto& c : a.solution) {
          std::cout << c << std::endl;
        }
      }

      return false;
    }
  
  
  private:
    FlatlandCBS& m_flatlandCBS;

    typedef AStar<FlatlandCBS, State, Agent, Action> AStar_t;
};
