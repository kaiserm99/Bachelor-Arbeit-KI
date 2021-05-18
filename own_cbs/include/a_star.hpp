#pragma once

#include <unordered_map>
#include <boost/heap/fibonacci_heap.hpp>

template <typename FlatlandCBS, typename State, typename Agent, typename Action, typename StateHasher = std::hash<State> >
class AStar {
  public:
    AStar(FlatlandCBS& flatlandCBS) : m_flatlandCBS(flatlandCBS) {}
    
    bool search(Agent& agent, int initialCost = 0) {
      // Clear the current solution and insert the initial state to the solution
      agent.solution.clear();
      agent.solution.emplace_back(agent.initialState);

      agent.soulutionCost = 0;

      // Create all the needed Data Structures
      openSet_t openSet;
      std::unordered_map<State, fibHeapHandle_t, StateHasher> stateToHeap;
      std::unordered_set<State, StateHasher> closedSet;
      std::unordered_map<State, std::tuple<State, Action, int, int>, StateHasher> cameFrom;


      // Insert the first Node into the Heap
      auto handle = openSet.push(Node(agent.initialState, agent.getHeuristicValue(agent.initialState), initialCost));
      stateToHeap.insert(std::make_pair<>(agent.initialState, handle));
      (*handle).handle = handle;

      std::vector<Neighbor <Action> > neighbors;
      neighbors.reserve(10);


      while (!openSet.empty()) {

        Node current = openSet.top();

        m_flatlandCBS.onExpandNode();

        if (agent.isSolution(current.state)) {
          agent.solution.clear();

          auto iter = cameFrom.find(current.state);

          while (iter != cameFrom.end()) {
            agent.solution.emplace_back(iter->first);
            iter = cameFrom.find(std::get<0>(iter->second));
          }

          agent.solution.emplace_back(agent.initialState);
          std::reverse(agent.solution.begin(), agent.solution.end());
          agent.soulutionCost = current.gScore;

          return true;
        }

        openSet.pop();
        stateToHeap.erase(current.state);
        closedSet.insert(current.state);

        m_flatlandCBS.getNeighbors(current.state, neighbors);

        for (const Neighbor <Action>& neighbor : neighbors) {
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