#pragma once

#include <boost/heap/fibonacci_heap.hpp>
#include <boost/heap/d_ary_heap.hpp>

#include <neighbor.hpp>
#include <planresult.hpp>


template <typename FlatlandCBS, typename State, typename Agent, typename Action, typename StateHasher = std::hash<State> >
class AStar {
  public:
    AStar(FlatlandCBS& flatlandCBS, const Constraints* constraints) : m_flatlandCBS(flatlandCBS) {
      m_flatlandCBS.setConstraints(constraints);
    }
    
    bool search(Agent& agent, PlanResult<Action, State>& solution, int initialCost = 0) {

      // Create all the needed Data Structures
      openSet_t openSet;
      std::unordered_map<State, fibHeapHandle_t, StateHasher> stateToHeap;
      std::unordered_set<State, StateHasher> closedSet;
      std::unordered_map<State, std::tuple<State, Action, int, int>, StateHasher> cameFrom;


      // Insert the first Node into the Heap
      auto handle = openSet.push(Node(agent.initialState, agent.getHeuristicValue(agent.initialState), initialCost));
      stateToHeap.insert(std::make_pair<>(agent.initialState, handle));
      (*handle).handle = handle;

      std::vector<Neighbor <Action, State> > neighbors;
      neighbors.reserve(10);


      while (!openSet.empty()) {

        Node current = openSet.top();

        m_flatlandCBS.onExpandNode();

        if (agent.isSolution(current.state)) {
          solution.states.clear();
          solution.actions.clear();

          auto iter = cameFrom.find(current.state);

          std::vector<State> tmpStates;

          while (iter != cameFrom.end()) {
          
            tmpStates.emplace_back(iter->first);
            solution.actions.push_back(std::make_pair<>(std::get<1>(iter->second), std::get<2>(iter->second)));
            iter = cameFrom.find(std::get<0>(iter->second));
          }

          tmpStates.emplace_back(agent.initialState);

          std::reverse(tmpStates.begin(), tmpStates.end());

          int time = 0;
          for (const auto& state : tmpStates) {
            
            if (&state == &tmpStates.back()) break;

            solution.states.emplace_back(state.setTime(time));

            for (int i = 1; i < agent.speed; i++) {
              solution.states.emplace_back(solution.states.back().timePlusT(1));
            }
            time += agent.speed;
          }



          
          std::reverse(solution.actions.begin(), solution.actions.end());

          solution.cost = solution.states.size();

          return true;
        }

        openSet.pop();
        stateToHeap.erase(current.state);
        closedSet.insert(current.state);

        m_flatlandCBS.getNeighbors(current.state, agent.speed, neighbors);

        for (const Neighbor <Action, State>& neighbor : neighbors) {
          if (closedSet.find(neighbor.state) == closedSet.end()) {
            int tentative_gScore = current.gScore + neighbor.cost;
            auto iter = stateToHeap.find(neighbor.state);

            if (iter == stateToHeap.end()) {

              int fScore = tentative_gScore + agent.getHeuristicValue(neighbor.state);
              auto handle = openSet.push(Node(neighbor.state, fScore, tentative_gScore));
              (*handle).handle = handle;
              stateToHeap.insert(std::make_pair<>(neighbor.state, handle));

            } else {

              auto handle = iter->second;
              if (tentative_gScore >= (*handle).gScore) continue;
              int delta = (*handle).gScore - tentative_gScore;
              (*handle).gScore = tentative_gScore;
              (*handle).fScore -= delta;
              openSet.increase(handle);
            }

            cameFrom.erase(neighbor.state);
            cameFrom.insert(std::make_pair<>(neighbor.state, std::make_tuple<>(current.state, neighbor.action, neighbor.cost, tentative_gScore)));

          }
        }
      }

      return false;
    }


  private:
    FlatlandCBS& m_flatlandCBS;

    struct Node {

      Node(const State& state, int fScore, int gScore) : state(state), fScore(fScore), gScore(gScore) {}

      State state;
      int fScore;
      int gScore;

      bool operator<(const Node& other) const {
        if (fScore != other.fScore) return fScore > other.fScore;
        else return gScore < other.gScore;
      }

      typename boost::heap::fibonacci_heap<Node>::handle_type handle;
    };

    typedef typename boost::heap::fibonacci_heap<Node> openSet_t;
    typedef typename openSet_t::handle_type fibHeapHandle_t;
};