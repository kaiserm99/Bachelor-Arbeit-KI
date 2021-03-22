#pragma once

#include <map>

#include "a_star.hpp"

namespace libMultiRobotPlanning {

/*!
  \example cbs.cpp Example that solves the Multi-Agent Path-Finding (MAPF)
  problem in a 2D grid world with up/down/left/right
  actions
*/

/*! \brief Conflict-Based-Search (CBS) algorithm to solve the Multi-Agent
Path-Finding (MAPF) problem

This class implements the Conflict-Based-Search (CBS) algorithm.
This algorithm can find collision-free path for multiple agents with start and
goal locations
given for each agent.
CBS is a two-level search. On the low-level, A* is used to find paths for
individual agents (ideally using a perfect heuristic).
The high-level is a tree-search that resolves conflicts between agents as they
occur, earliest conflict-time first.
CBS is optimal with respect to the sum-of-individual costs.

Details of the algorithm can be found in the following paper:\n
Guni Sharon, Roni Stern, Ariel Felner, Nathan R. Sturtevant:\n
"Conflict-based search for optimal multi-agent pathfinding". Artif. Intell. 219:
40-66 (2015)\n
https://doi.org/10.1016/j.artint.2014.11.006

The underlying A* can either use a fibonacci heap, or a d-ary heap.
The latter is the default. Define "USE_FIBONACCI_HEAP" to use the fibonacci heap
instead.

\tparam State Custom state for the search. Needs to be copy'able
\tparam Action Custom action for the search. Needs to be copy'able
\tparam Cost Custom Cost type (integer or floating point types)
\tparam Conflict Custom conflict description. A conflict needs to be able to be
transformed into a constraint.
\tparam Constraints Custom constraint description. The Environment needs to be
able to search on the low-level while taking the constraints into account.
\tparam Environment This class needs to provide the custom logic. In particular,
it needs to support the following functions:
  - `void setLowLevelContext(size_t agentIdx, const Constraints* constraints)`\n
    Set the current context to a particular agent with the given set of
constraints

  - `Cost admissibleHeuristic(const State& s)`\n
    Admissible heuristic. Needs to take current context into account.

  - `bool isSolution(const State& s)`\n
    Return true if the given state is a goal state for the current agent.

  - `void getNeighbors(const State& s, std::vector<Neighbor<State, Action, int>
>& neighbors)`\n
    Fill the list of neighboring state for the given state s and the current
agent.

  - `bool getFirstConflict(const std::vector<PlanResult<State, Action, int> >&
solution, Conflict& result)`\n
    Finds the first conflict for the given solution for each agent. Return true
if a conflict was found and false otherwise.

  - `void createConstraintsFromConflict(const Conflict& conflict,
std::map<size_t, Constraints>& constraints)`\n
    Create a list of constraints for the given conflict.

  - `void onExpandHighLevelNode(Cost cost)`\n
    This function is called on every high-level expansion and can be used for
statistical purposes.

  - `void onExpandLowLevelNode(const State& s, Cost fScore, Cost gScore)`\n
    This function is called on every low-level expansion and can be used for
statistical purposes.
*/
template <typename State, typename Action, typename Cost, typename Conflict,
          typename Constraints, typename Environment>
class CBS {
 public:
  CBS(Environment& environment) : m_env(environment) {}

  bool search(const std::vector<State>& initialStates,
              std::vector<PlanResult<State, Action, Cost> >& solution) {
    HighLevelNode start;
    start.solution.resize(initialStates.size());
    start.constraints.resize(initialStates.size());
    start.cost = 0;
    start.id = 0;


    // Zuerst wird auf alle Agenten der normale A-Stern Algorithmus ausgeführt, um zu überprüfuen,
    // ob es überhaupt eine Lösung gibt und um die jeweiligen inizalen Kosten der Lösung zu erstellen,
    // da im Folgenden die HighLevelNodes mit den maximalen Kosten zuerst geprüft werden

    for (size_t handle = 0; handle < initialStates.size(); ++handle) {

      // Es wird ein A-Stern env erzeugt, bei dem die constraints noch leer sind, auf die dann geschrieben werden kann
      LowLevelEnvironment llenv(m_env, handle, start.constraints[handle]);
      LowLevelSearch_t lowLevel(llenv);

      // Der eigentliche A-Stern Algorithmus wird aufgerufen und das Ergebnis wird in start.soloution[handle] gespeicher
      bool success = lowLevel.search(initialStates[handle], start.solution[handle]);
      if (!success) {
        return false;
      }


      start.cost += start.solution[handle].cost;
    }

    
    typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>, boost::heap::mutable_<true>> open;

    // Das erste start Node wird in den Heap eingefügt, damit im Folgenden weiter mit diesem gearbeitet werden kann
    auto h = open.push(start);
    (*h).handle = h;

    solution.clear();
    int id = 1;
    while (!open.empty()) {

      // Das HighLevelNode mit den höchsten Kosten wird immer zuerst expaniert und weiter geprüft
      HighLevelNode P = open.top();
      m_env.onExpandHighLevelNode(P.cost);  // Nur um die Statistiken zu tracken
      
      // std::cout << "expand: " << P << std::endl;  // DEBUG

      open.pop();

      // Die Konflikte werden erstellt und in der Funktion mit call by reference bearbeitet

      // std::vector<Conflict> conflict_vec;

      std::map<size_t, Constraints> resultConstraints;
      // Conflict conflict;
      if (!m_env.getFirstConflict(P.solution, resultConstraints)) {

        std::cout << "Final HighLevelNode:" << std::endl << P << std::endl << std::endl;

        // Wenn es keine Konflikte gab, dann wird die jeweilige Lösung auch zurückgegeben
        std::cout << "done; cost: " << P.cost << std::endl;
        solution = P.solution;
        return true;
      }


      
      // for (const auto& cons : resultConstraints) {
        // std::cout << cons.first << std::endl;
        // std::cout << cons.second << std::endl;
      // }
      

      /*
      std::cout << "Found conflict: " << conflict << std::endl;
      std::cout << "Found conflict at t=" << conflict.time << " type: " <<
      conflict.type << std::endl << std::endl;
      */


      // std::map<size_t, Constraints> constraints;
      // m_env.createConstraintsFromConflict(conflict, constraints);

      for (const auto& c : resultConstraints) {
        if (c.second.vertexConstraints.size() == 0) {
          continue;
        }


        // std::cout << "Add HL node for " << c.first << std::endl;
        size_t handle = c.first;
        // std::cout << "create child with id " << id << std::endl;
        HighLevelNode newNode = P;
        newNode.id = id;
        // (optional) check that this constraint was not included already
        // std::cout << newNode.constraints[handle] << std::endl;
        // std::cout << c.second << std::endl;

        // assert(!newNode.constraints[handle].overlap(c.second));

        newNode.constraints[handle].add(c.second);

        newNode.cost -= newNode.solution[handle].cost;

        LowLevelEnvironment llenv(m_env, handle, newNode.constraints[handle]);
        LowLevelSearch_t lowLevel(llenv);
        bool success = lowLevel.search(initialStates[handle], newNode.solution[handle]);

        newNode.cost += newNode.solution[handle].cost;

        // std::cout << "HL Node, after search:" << std::endl;
        // std::cout << newNode << std::endl;

        if (success) {
          // std::cout << "  success. cost: " << newNode.cost << std::endl;
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
    std::vector<PlanResult<State, Action, Cost> > solution;
    std::vector<Constraints> constraints;

    Cost cost;

    int id;

    typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                     boost::heap::mutable_<true> >::handle_type
        handle;

    bool operator<(const HighLevelNode& n) const {
      // if (cost != n.cost)
      return cost > n.cost;
      // return id > n.id;
    }

    friend std::ostream& operator<<(std::ostream& os, const HighLevelNode& c) {
      os << "id: " << c.id << " cost: " << c.cost << std::endl;
      for (size_t handle = 0; handle < c.solution.size(); ++handle) {
        os << "Agent: " << handle << std::endl;
        os << " States:" << std::endl;
        for (size_t t = 0; t < c.solution[handle].states.size(); ++t) {
          os << "  " << c.solution[handle].states[t].first << std::endl;
        }
        os << " Constraints:" << std::endl;
        os << c.constraints[handle];
        os << " cost: " << c.solution[handle].cost << std::endl;
      }
      return os;
    }
  };

  struct LowLevelEnvironment {
    LowLevelEnvironment(Environment& env, size_t agentIdx,
                        const Constraints& constraints)
        : m_env(env)
    // , m_agentIdx(agentIdx)
    // , m_constraints(constraints)
    {
      m_env.setLowLevelContext(agentIdx, &constraints);
    }

    Cost admissibleHeuristic(const State& s) {
      return m_env.admissibleHeuristic(s);
    }

    bool isSolution(const State& s) { return m_env.isSolution(s); }

    void getNeighbors(const State& s,
                      std::vector<Neighbor<State, Action, Cost> >& neighbors) {
      m_env.getNeighbors(s, neighbors);
    }

    void onExpandNode(const State& s, Cost fScore, Cost gScore) {
      // std::cout << "LL expand: " << s << std::endl;
      m_env.onExpandLowLevelNode(s, fScore, gScore);
    }

    void onDiscover(const State& /*s*/, Cost /*fScore*/, Cost /*gScore*/) {
      // std::cout << "LL discover: " << s << std::endl;
      // m_env.onDiscoverLowLevel(s, m_agentIdx, m_constraints);
    }

   private:
    Environment& m_env;
    // size_t m_agentIdx;
    // const Constraints& m_constraints;
  };

 private:
  Environment& m_env;
  typedef AStar<State, Action, Cost, LowLevelEnvironment> LowLevelSearch_t;
};

}  // namespace libMultiRobotPlanning
