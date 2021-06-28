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
    
    bool search(Agent& agent, PlanResult<Action, State>& solution, bool initialSearch = false, int initialCost = 0) {
      solution.states.clear();
      solution.actions.clear();

      // Create all the needed Data Structures
      openSet_t openSet;
      std::unordered_map<State, fibHeapHandle_t, StateHasher> stateToHeap;
      std::unordered_set<State, StateHasher> closedSet;
      std::unordered_map<State, std::tuple<State, Action, int, int>, StateHasher> cameFrom;

      int initialConstraintEndTime = m_flatlandCBS.getInitialConstraintEndTime();

      

      if (m_flatlandCBS.checkInitialConstraints()) {
        int tmp = m_flatlandCBS.getHighestEndingTime(State(initialConstraintEndTime, agent.initialState));
        if (tmp != -1) initialConstraintEndTime = tmp;
        agent.initialState = State(initialConstraintEndTime+1, agent.initialState.y, agent.initialState.x, agent.initialState.dir);
      }

      // Insert the first Node into the Heap
      auto handle = openSet.push(Node(agent.initialState, agent.getHeuristicValue(agent.initialState), initialCost));
      stateToHeap.insert(std::make_pair<>(agent.initialState, handle));
      (*handle).handle = handle;

      std::vector<Neighbor <Action, State> > neighbors;
      neighbors.reserve(10);


      while (!openSet.empty()) {

        Node current = openSet.top();

        // std::cout << current << std::endl;

        if (initialSearch) m_flatlandCBS.onExpandInitialNode();
        else m_flatlandCBS.onExpandNode();

        if (agent.isSolution(current.state)) {
          solution.states.clear();
          solution.actions.clear();

          auto iter = cameFrom.find(current.state);

          std::vector<State> tmpStates;
          std::vector<std::pair <Action, int> > tmpActions;

          while (iter != cameFrom.end()) {
          
            tmpStates.emplace_back(iter->first);
            solution.actions.push_back(std::make_pair<>(std::get<1>(iter->second), std::get<2>(iter->second)));
            iter = cameFrom.find(std::get<0>(iter->second));
          }

          tmpStates.emplace_back(agent.initialState);

          std::reverse(tmpStates.begin(), tmpStates.end());
          std::reverse(solution.actions.begin(), solution.actions.end());

          int time;

          // If there is a inital state, insert the default value
          for (time = 0; time <= initialConstraintEndTime; time++) {
            solution.states.emplace_back(defaultState);
            tmpActions.emplace_back(std::make_pair(Action::Nothing, 0));
          }

          // Initial action to place the agent on the grid
          tmpActions.emplace_back(Action::Forward, 0);

          // Insert the other states and actions into the vectors
          int index = 0;
          for (const auto& state : tmpStates) {
            
            if (&state == &tmpStates.back()) break;

            solution.states.emplace_back(state.setTime(time));
            tmpActions.emplace_back(solution.actions[index]);

            // If there is a wait action at this state, we do not replicate it
            if (solution.actions[index].first == Action::Wait) {
              for (int i = 1; i < agent.speed; i++) {
                tmpActions.emplace_back(solution.actions[index]);
              } 
            }
            
            // Add more states, based on the agent speed
            for (int i = 1; i < agent.speed; i++) {
              solution.states.emplace_back(solution.states.back().timePlusT(1));
            } 

            time += agent.speed;
            index++;
          }

          // Append the taraget state
          solution.states.emplace_back(tmpStates.back());

          solution.actions = tmpActions;
          solution.cost = solution.states.size();

          return true;
        }

        openSet.pop();
        stateToHeap.erase(current.state);
        closedSet.insert(current.state);

        m_flatlandCBS.getNeighbors(current.state, agent, neighbors);

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

      friend std::ostream& operator<<(std::ostream& os, const Node& node) {
        os << "state: " << node.state << " fScore: " << node.fScore << " gScore: " << node.gScore;
        return os;
      }

      State state;
      int fScore;
      int gScore;

      bool operator<(const Node& other) const {
        if (fScore != other.fScore) return fScore > other.fScore;
        else return gScore < other.gScore;
      }

      typename boost::heap::fibonacci_heap<Node>::handle_type handle;
      // typename boost::heap::d_ary_heap<Node, boost::heap::arity<2>, boost::heap::mutable_<true> >::handle_type handle;
    };

    typedef typename boost::heap::fibonacci_heap<Node> openSet_t;
    // typedef typename boost::heap::d_ary_heap<Node, boost::heap::arity<2>, boost::heap::mutable_<true> > openSet_t;
    typedef typename openSet_t::handle_type fibHeapHandle_t;
};