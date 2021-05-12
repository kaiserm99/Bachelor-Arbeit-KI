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

  - `bool getFirstConflict(const std::vector<PlanResult<State, Action, sizt_t> >&
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
  CBS (Environment& environment) : m_env(environment) {}  

  bool search(const std::vector<State>& initialStates, const std::vector<int>& speeds, std::vector<PlanResult<State, Action, int>>& solution, std::vector<Constraints>& constraints, std::vector<int>& replanNextAssignment, bool replan) {
    

    HighLevelNode start;
    start.solution.resize(initialStates.size());

    
    std::vector<int> replannigSizes;

    if (replan) {

      start.constraints = constraints;

      for (const auto& con : constraints) {
        replannigSizes.emplace_back(con.vertexConstraints.size());
      }

    }

    else {

      replannigSizes.resize(initialStates.size(), 0);
      start.constraints.resize(initialStates.size());
      replanNextAssignment.resize(initialStates.size(), 0);
    }    


    std::cout << "Print the given constraints" << std::endl;
    for (int handle = 0; (unsigned) handle < start.constraints.size(); handle++) {
      std::cout << "Agent " << handle << std::endl;
      
      for (const auto& a : start.constraints[handle].vertexConstraints) {
       std::cout << "  " << a << std::endl;
      }
    }

    std::cout << "Print the starting states" << std::endl;
    for (const auto& asdf : initialStates) {
       std::cout << asdf << std::endl;
    }

    start.cost = 0;
    start.id = 0;


    for (size_t handle = 0; handle < initialStates.size(); ++handle) {

      // Es wird ein A-Stern env erzeugt, bei dem die constraints noch leer sind, auf die dann geschrieben werden kann
      LowLevelEnvironment llenv(m_env, handle, speeds[handle], replannigSizes[handle], start.constraints[handle]);
      LowLevelSearch_t lowLevel(llenv);


      if (initialStates[handle].time != -1) {
        if (!lowLevel.search(initialStates[handle], start.solution[handle], speeds[handle], replannigSizes[handle], replanNextAssignment[handle])) return false;
        start.cost += start.solution[handle].cost;
      }

      
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
      
      std::cout << "expand Node ID: " << P.id << ", with the cost of: " << P.cost << std::endl;  // DEBUG

      open.pop();

      // Die Konflikte werden erstellt und in der Funktion mit call by reference bearbeitet

      // std::vector<Conflict> conflict_vec;

      std::vector<std::pair<size_t, Constraints>> resultConstraints;

      std::map<std::pair<size_t, size_t>, std::pair<Constraints, Constraints>> resultDoubleConstraints;
      std::map<std::pair<size_t, size_t>, std::pair<Constraints, Constraints>> resultDeleteConstraints;

      // Conflict conflict;
      if (!m_env.getFirstConflict(P.solution, resultConstraints, resultDoubleConstraints, resultDeleteConstraints, P.constraints)) {

        // std::cout << "Final HighLevelNode:" << std::endl << P << std::endl << std::endl;

        // Wenn es keine Konflikte gab, dann wird die jeweilige Lösung auch zurückgegeben
        std::cout << "done; cost: " << P.cost << std::endl;
        solution = P.solution;

        saveHighLevelNode(P.constraints);
        return true;
      }


      for (const auto& c : resultDeleteConstraints) {

        // std::cout << "Searching with Delete Constraints!" << std::endl;

        size_t handle1 = c.first.first;
        size_t handle2 = c.first.second;

        HighLevelNode newNode = P;
        newNode.id = id;

        newNode.constraints[handle1].add(c.second.first);
        newNode.constraints[handle2].del(c.second.second);

        if (searchLowLevel(newNode, handle1, speeds, initialStates, replannigSizes, replanNextAssignment) && searchLowLevel(newNode, handle2, speeds, initialStates, replannigSizes, replanNextAssignment)) {
          // std::cout << "Cost of the delete Node " << newNode.id << ": " << newNode.cost << std::endl;
          auto h = open.push(newNode);
          (*h).handle = h;
        }

        ++id;
      }


      for (const auto& c : resultDoubleConstraints) {

        // std::cout << "Searching with Double Constraints!" << std::endl;

        size_t handle1 = c.first.first;
        size_t handle2 = c.first.second;

        HighLevelNode newNode = P;
        newNode.id = id;

        newNode.constraints[handle1].add(c.second.first);
        newNode.constraints[handle2].add(c.second.second);

        /*
        for (const auto& c : c.second.first.vertexConstraints) {
          std::cout << c << std::endl;
        }

        for (const auto& c : c.second.second.vertexConstraints) {
          std::cout << c << std::endl;
        }
        */

        if (searchLowLevel(newNode, handle1, speeds, initialStates, replannigSizes, replanNextAssignment) && searchLowLevel(newNode, handle2, speeds, initialStates, replannigSizes, replanNextAssignment)) {
          // std::cout << "Cost of the double Node " << newNode.id << ": " << newNode.cost << std::endl;
          auto h = open.push(newNode);
          (*h).handle = h;
        }

        ++id;
      }


      

      


      for (const auto& c : resultConstraints) {
        if (c.second.vertexConstraints.size() == 0) {
          continue;
        }

        size_t handle = c.first;

        HighLevelNode newNode = P;
        newNode.id = id;

        newNode.constraints[handle].add(c.second);

        /*
        for (const auto& c : c.second.vertexConstraints) {
          std::cout << c << std::endl;
        }
        */

        if (searchLowLevel(newNode, handle, speeds, initialStates, replannigSizes, replanNextAssignment)) {
          // std::cout << "Cost of the normal Node " << newNode.id << ": " << newNode.cost << std::endl;
          auto h = open.push(newNode);
          (*h).handle = h;
        }

        ++id;
      }
    }

    return false;
  }

  bool searchLowLevel(auto& newNode, size_t handle, const auto& speeds, const auto& initialStates, const auto& replannigSizes, const auto& replanNextAssignment) {

    newNode.cost -= newNode.solution[handle].cost;

    LowLevelEnvironment llenv(m_env, handle, speeds[handle], replannigSizes[handle], newNode.constraints[handle]);
    LowLevelSearch_t lowLevel(llenv);
    std::cout << "" << replanNextAssignment[handle] << std::endl;
    bool success = lowLevel.search(initialStates[handle], newNode.solution[handle], speeds[handle], replannigSizes[handle], replanNextAssignment[handle]);

    /*
    std::cout << "New Solution for " << handle << std::endl;
    for (const auto& s : newNode.solution[handle].states) {
    std::cout << s.first << std::endl;
    }
    */

    newNode.cost += newNode.solution[handle].cost;

    return success;
  }

  void saveHighLevelNode(std::vector<Constraints> constraints) {

    std::ofstream out("replan.yaml");

    out << "agents: " << std::endl;
    for (size_t handle = 0; handle < constraints.size(); handle++) {

      out << "- handle: " << handle << std::endl;
    
      out << "  constraints:";

      if (constraints[handle].vertexConstraints.size() == 0) {
        out << " []" << std::endl;
        continue;
      }

      out << std::endl;

      for (const auto& con : constraints[handle].vertexConstraints) {
        out << "     - t : " << con.time << std::endl
        <<     "       y : " << con.y << std::endl
        <<     "       x : " << con.x << std::endl
        <<     "       sf : " << con.setFrom << std::endl;
      }
    }

  }

 private:
  struct HighLevelNode {
    std::vector<PlanResult<State, Action, int> > solution;
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
    LowLevelEnvironment(Environment& env, size_t agentIdx, int speed, int replanningSize, const Constraints& constraints)
        : m_env(env)
    // , m_agentIdx(agentIdx)
    // , m_constraints(constraints)
    {
      m_env.setLowLevelContext(agentIdx, speed, replanningSize, &constraints);
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
    
  };

 private:
  Environment& m_env;
  // HighLevelNode& m_highLevelNode = nullptr;
  typedef AStar<State, Action, Cost, LowLevelEnvironment> LowLevelSearch_t;
};

}  // namespace libMultiRobotPlanning
