#include <fstream>
#include <iostream>
#include <sstream>


#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

#include <yaml-cpp/yaml.h>

#include <libMultiRobotPlanning/cbs.hpp>
#include "timer.hpp"

using libMultiRobotPlanning::CBS;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;


class Map {
  public:
    Map(std::string edge_file, std::string grid_file) :
      // The name of the files to generate the possible movements and the grid for the heuristic function
      m_edge_file(edge_file),
      m_grid_file(grid_file)

    {
      m_agent_count = 0;
      m_dimx = 0;
      m_dimy = 0;
      m_grid_data = 0;

    }

    // Somehow doesn't work....
    ~Map() {
      // delete[] m_grid_data;
    }


    void inizialize() {

      // Read the grid of the file
      std::ifstream infile(m_grid_file);


      std::string line;
      std::getline(infile, line);

      // Start at 1 so we don't read the [ of the file
      size_t acc = 1;
      std::string acc_string = "";

      // At fist set the numbers of the agent
      while (std::string(1, line[acc]) != ",") {
        acc_string += line[acc];
        acc++;
      }

      acc++; acc++;  // leap over comma and space
      m_agent_count = std::stoi(acc_string);

      // dimy
      acc_string = "";
      while (std::string(1, line[acc]) != ",") {
        acc_string += line[acc];
        acc++;
      }

      acc++; acc++;  // leap over the comma and space
      m_dimy = std::stoi(acc_string);

      // dimx
      acc_string = "";
      while (std::string(1, line[acc]) != ",") {
        acc_string += line[acc];
        acc++;
      }

      m_dimx = std::stoi(acc_string);

      // Allocate the needed memory, based on the read shape
      m_grid_data = new int[m_agent_count * m_dimy * m_dimx * 4];


      std::cout << "Creating a grid with shape: " << m_agent_count << " " << m_dimy << " " << m_dimx << std::endl;


      // Make sure to reset the acc and read the data of the grid
      acc = 0;
      std::getline(infile, line);

      for (int i = 0; i < m_agent_count * m_dimx * m_dimy * 4; i++) {

        acc_string = "";
        while (std::string(1, line[acc]) != " ") {
          acc_string += line[acc];
          acc++;
        }
        acc++;  // make sure the next " " is also skipped 

        m_grid_data[i] = std::stoi(acc_string);
      }
    }


    int getHeuristicValue(int handle, int posy, int posx, int dir) {
      
      // array[agents_count, dimy, dimx, 4] 

      int value = (((handle * m_dimy + posy) * m_dimx + posx) * 4 + dir);

      return m_grid_data[value];
    }


    void printFullGridData() {
      // Loop trough the whole map and print it based on the given shape
      for (int handle = 0; handle < m_agent_count; ++handle) {
        for (int y = 0; y < m_dimy; ++y) {
          for (int x = 0; x < m_dimx; ++x) {
            for (int dir = 0; dir < 4; ++dir) {

              std::cout << handle << " "<< y << " "<< x << " " << dir << " " 
              << getHeuristicValue(handle, y, x, dir) << std::endl;

            } 
          }
        }
      }
    }

  private:
    // Shape variables
    int m_agent_count;
    int m_dimy;
    int m_dimx;

    // Filenames
    std::string m_edge_file;
    std::string m_grid_file;

    // Actual data of the grid saved in a one dimensional array
    int* m_grid_data;
};









struct State {
  State(int time, int y, int x, int dir) : time(time), y(y), x(x), dir(dir) {}

  bool operator==(const State& s) const {
    return time == s.time && x == s.x && y == s.y;
  }

  bool equalExceptTime(const State& s) const { return x == s.x && y == s.y; }

  friend std::ostream& operator<<(std::ostream& os, const State& s) {
    return os << s.time << ": (" << s.y << "," << s.x << "," << s.dir << ")";
  }

  int time;
  int y;
  int x;
  int dir;
};

namespace std {
template <>
struct hash<State> {
  size_t operator()(const State& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
}  // namespace std

///
enum class Action {
  Up,
  Down,
  Left,
  Right,
  Wait,
};

std::ostream& operator<<(std::ostream& os, const Action& a) {
  switch (a) {
    case Action::Up:
      os << "2";
      break;
    case Action::Down:
      os << "Should never trigger!";
      break;
    case Action::Left:
      os << "1";
      break;
    case Action::Right:
      os << "3";
      break;
    case Action::Wait:
      os << "4";
      break;
  }
  return os;
}

///

struct Conflict {
  enum Type {
    Vertex,
    Edge,
  };

  int time;
  size_t agent1;
  size_t agent2;
  Type type;

  int y1;
  int x1;
  int y2;
  int x2;

  friend std::ostream& operator<<(std::ostream& os, const Conflict& c) {
    switch (c.type) {
      case Vertex:
        return os << c.time << ": Vertex(" << c.y1 << "," << c.x1 << ")";
      case Edge:
        return os << c.time << ": Edge(" << c.y1 << "," << c.x1 << "," << c.y2
                  << "," << c.x2 << ")";
    }
    return os;
  }
};

struct VertexConstraint {
  VertexConstraint(int time, int y, int x) : time(time), y(y), x(x) {}
  int time;
  int y;
  int x;

  bool operator<(const VertexConstraint& other) const {
    return std::tie(time, x, y) < std::tie(other.time, other.x, other.y);
  }

  bool operator==(const VertexConstraint& other) const {
    return std::tie(time, x, y) == std::tie(other.time, other.x, other.y);
  }

  friend std::ostream& operator<<(std::ostream& os, const VertexConstraint& c) {
    return os << "VC(t=" << c.time << ",y=" << c.y << ",x=" << c.x << ")";
  }
};

namespace std {
template <>
struct hash<VertexConstraint> {
  size_t operator()(const VertexConstraint& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
}  // namespace std

struct Constraints {
  std::unordered_set<VertexConstraint> vertexConstraints;

  void add(const Constraints& other) {
    vertexConstraints.insert(other.vertexConstraints.begin(),
                             other.vertexConstraints.end());
  }


  friend std::ostream& operator<<(std::ostream& os, const Constraints& c) {
    for (const auto& vc : c.vertexConstraints) {
      os << vc << std::endl;
    }

    return os;
  }
};


struct NewLocation {
  NewLocation(int y, int x, int new_dir, int action) : y(y), x(x), new_dir(new_dir), action(action) {}
  int y;
  int x;
  int new_dir;
  int action;

  friend std::ostream& operator<<(std::ostream& os, const NewLocation& c) {
    return os << "(y:" << c.y << ", x:" << c.x << ", nd:" << c.new_dir << ", a:" << c.action << ")";    
  }
};


struct Location {
  Location(int y, int x, int dir=-1) : y(y), x(x), dir(dir) {}
  int y;
  int x;
  int dir;

  bool operator<(const Location& other) const {
    std::cout << "YOU NEED TO DEBUG!" << std::endl;
    return std::tie(x, y) < std::tie(other.x, other.y);
  }

  bool operator==(const Location& other) const {
    if (dir == -1 || other.dir == -1) {
      std::cout << "old == Operator in Location" << std::endl;

      return std::tie(x, y) == std::tie(other.x, other.y);
    }

    return std::tie(x, y, dir) == std::tie(other.x, other.y, other.dir);
  }

  friend std::ostream& operator<<(std::ostream& os, const Location& c) {
    if (c.dir == -1) {
      return os << "(" << c.x << "," << c.y << ")";
    }

    return os << "(" << c.x << "," << c.y << "," << c.dir << ")";    
  }
};

namespace std {
template <>
struct hash<Location> {
  size_t operator()(const Location& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    boost::hash_combine(seed, s.dir);
    return seed;
  }
};
}  // namespace std

///
class Environment {
 public:
  Environment(size_t dimy, size_t dimx, std::unordered_map<Location, std::vector<NewLocation>>  edges,
              std::vector<Location> goals, Map map, std::vector<Location> critical)
      : m_dimy(dimy),
        m_dimx(dimx),
        m_edges(std::move(edges)),
        m_goals(std::move(goals)),
        m_map(map),
        m_critical(critical),
        m_agentIdx(0),
        m_constraints(nullptr),
        m_lastGoalConstraint(-1),
        m_highLevelExpanded(0),
        m_lowLevelExpanded(0) {
  }

  Environment(const Environment&) = delete;
  Environment& operator=(const Environment&) = delete;

  void setLowLevelContext(size_t agentIdx, const Constraints* constraints) {
    assert(constraints);  // NOLINT
    m_agentIdx = agentIdx;
    m_constraints = constraints;
    m_lastGoalConstraint = -1;
    for (const auto& vc : constraints->vertexConstraints) {
      if (vc.x == m_goals[m_agentIdx].x && vc.y == m_goals[m_agentIdx].y) {
        m_lastGoalConstraint = std::max(m_lastGoalConstraint, vc.time);
      }
    }
  }

  int admissibleHeuristic(const State& s) {
    return m_map.getHeuristicValue(m_agentIdx, s.y, s.x, s.dir);
  }

  bool isSolution(const State& s) {
    return s.x == m_goals[m_agentIdx].x && s.y == m_goals[m_agentIdx].y && s.time > m_lastGoalConstraint;
  }

  void getNeighbors(const State& s,
                    std::vector<Neighbor<State, Action, int> >& neighbors) {

    neighbors.clear();

    {
      // Always append the do_nothing (4) State if valid
      State n(s.time + 1, s.y, s.x, s.dir);

      if (stateValid(n)) {
        neighbors.emplace_back(Neighbor<State, Action, int>(n, Action::Wait, 1));
      }

    }
  
    {
      // Get all the next actions which are reachable from the current position and direction
      const auto& next_actions = m_edges[Location(s.y, s.x, s.dir)];

      // Loop trough all the possible next actions and check if they are valid with the other ones
      for (const auto& next_action : next_actions) {

        // Create the new State, which is checked in the following
        State n(s.time + 1, next_action.y, next_action.x, next_action.new_dir);


        if (stateValid(n)) {

          // std::cout << "Agent: " << m_agentIdx << " Von: " << s << std::endl;
          // std::cout << "Zu : " << next_action << std::endl;

          switch (next_action.action) {
            case 1: neighbors.emplace_back(Neighbor<State, Action, int>(n, Action::Left, 1)); break;

            case 2: neighbors.emplace_back(Neighbor<State, Action, int>(n, Action::Up, 1)); break;

            case 3: neighbors.emplace_back(Neighbor<State, Action, int>(n, Action::Right, 1)); break;
          }
        }
      }
    }
  }


  // More for optimization 
  bool checkCriticalLocation(State state) {

    for (const auto& crit: m_critical) {
      if (state.x == crit.x && state.y == crit.y) {
        return true;
      }

    }

    return false;
  }

  State getCriticalState(int t, size_t handle1, size_t handle2, const std::vector<PlanResult<State, Action, int> >& solution) {

    int t_back;

    for (t_back = t; t_back >= 0; t_back--) {

      State checkState = getState(handle1, solution, t_back);
      
      if (!checkCriticalLocation(checkState)) {
        continue;  // The current state position is not a critical Point
      }

      // If the critical Point is the goal of the other agent, then return this one
      if (checkState.x == m_goals[handle2].x && checkState.y == m_goals[handle2].y && t_back != 0) {
        // std::cout << "TEST: Agent " << handle1 << " lässt Agent "<< handle2 << " durch" << std::endl;
        return checkState;
      }

      // If it is a critical state, then check if minimum one of the agents can use it
      if (m_edges[Location(checkState.y, checkState.x, checkState.dir)].size() >= 2) {
        return checkState;  
      }


      if (m_edges[Location(checkState.y, checkState.x, (checkState.dir + 2) % 4)].size() >= 2) {
        return checkState;  
      }

      // If there was no return statement and both of the agents can't use the critical point, go to the other critical point

    }

    return State(-1, -1, -1, -1);
  }


  // type = 1 --> the found critical cell, can be used otherwise by the agent
  // type = 0 --> not
  void addConstraints(int t, size_t handle1, size_t handle2, State& criticalState, auto& solution, Constraints& cons, Constraints& consOther, int type, int edge = 0) {


    if (type == 1) {

      handleApplicableJunction(t, criticalState, handle1, handle2, edge, cons, solution);
    }


    else if (type == 0) {

      handleNotApplicableJunction(t, criticalState, handle1, handle2, edge, cons, solution);
    }


    return;
  }



  void handleConflicts(const State& errorState1, const State& errorState2, const int errorTime, const size_t handle1, const size_t handle2, const auto& solution, auto& resultConstraints, int edge) {


    if (edge == 0) {
      if (m_debug) std::cout << std::endl << "[t=" << errorTime << "] Vertex Error at: " << errorState1 << " and " << errorState2 << std::endl;
    } else {
      if (m_debug) std::cout << std::endl << "[t=" << errorTime << "] Edge Error at: " << errorState1 << " and " << errorState2 << std::endl;
    }

    // Get to the critical states
    State criticalState1 = getCriticalState(errorTime, handle1, handle2, solution);

    State criticalState2 = getCriticalState(errorTime, handle2, handle1, solution);

    if (m_debug) std::cout << "Agent: " << handle1 << ", Critical State: " << criticalState1 << std::endl;
    if (m_debug) std::cout << "Agent: " << handle2 << ", Critical State: " << criticalState2 << std::endl;


    Constraints accConstraints1;
    Constraints accConstraints2;


    // Check if the the error states of the Agents is a Vertex error and if they have the same direction, then one have to wait for the other
    if (errorState1.dir == errorState2.dir  && edge == 0) {

      if (true) std::cout << "Aufgefahren!" << std::endl;

      // Agent1 is waiting for another so, the previous agent has to wait
      if (errorState1.equalExceptTime(solution[handle1].states[errorState1.time-1].first)) {

        // Loop troth the vertex error time and check how long the agent has to wait and add the constraints to the other agents
        for (int waitTime = errorTime; (unsigned)waitTime < solution[handle1].states.size(); waitTime++) {

          if (!solution[handle1].states[waitTime].first.equalExceptTime(errorState2)) break;

          accConstraints2.vertexConstraints.emplace(VertexConstraint(waitTime, errorState1.y, errorState1.x));
        }
      }

              
      if (errorState2.equalExceptTime(solution[handle2].states[errorState2.time-1].first)) {

        for (int waitTime = errorTime; (unsigned)waitTime < solution[handle2].states.size(); waitTime++) {

          if (!solution[handle2].states[waitTime].first.equalExceptTime(errorState1)) break;

          accConstraints1.vertexConstraints.emplace(VertexConstraint(waitTime, errorState2.y, errorState2.x));
        }
      }
    }


    // There is no way the agent can get over each other, so return true and don't expand this HighLevelNode further
    else if (criticalState1.time == -1 && criticalState2.time == -1) {
      if (m_debug) std::cout << "Agent: "<<  handle1 << " & Agent: " <<  handle2 << " can't pass each other!" << std::endl;
    }

    // Agent 1 cant get out of the situation 
    else if (criticalState1.time == -1) {

      if (m_edges[Location(criticalState2.y, criticalState2.x, criticalState2.dir)].size() >= 2) {

        if (m_debug) std::cout << "TEST 1a" << std::endl;

        addConstraints(errorTime, handle2, handle1, criticalState2, solution, accConstraints2, accConstraints1, 1, edge);


      } else {

        if (m_debug) std::cout << "TEST 1b" << std::endl;

        addConstraints(errorTime, handle2, handle1, criticalState2, solution, accConstraints2, accConstraints2, 0, edge);
      }
    }

    // Agent 2 cant get out of the situation 
    else if (criticalState2.time == -1) {

      if (m_edges[Location(criticalState1.y, criticalState1.x, criticalState1.dir)].size() >= 2) {

        if (m_debug) std::cout << "TEST 2a" << std::endl;
        addConstraints(errorTime, handle1, handle2, criticalState1, solution, accConstraints1, accConstraints2, 1, edge);

      } else {

        if (m_debug) std::cout << "TEST 2b" << std::endl;
        addConstraints(errorTime, handle1, handle2, criticalState1, solution, accConstraints1, accConstraints1, 0, edge);
      }
    } 

    // Both Agents can get out of the situation
    else {  

      // Constrain Agent 1
      if (m_edges[Location(criticalState1.y, criticalState1.x, criticalState1.dir)].size() >= 2) {

        if (m_debug) std::cout << "TEST 3a" << std::endl;
        addConstraints(errorTime, handle1, handle2, criticalState1, solution, accConstraints1, accConstraints2, 1, edge);

      } else {

        if (m_debug) std::cout << "TEST 3b" << std::endl;
        addConstraints(errorTime, handle1, handle2, criticalState1, solution, accConstraints1, accConstraints1, 0, edge);

      }

      // Constrain Agent 2
      if (m_edges[Location(criticalState2.y, criticalState2.x, criticalState2.dir)].size() >= 2) {

        if (m_debug) std::cout << "TEST 4a" << std::endl;
        addConstraints(errorTime, handle2, handle1, criticalState2, solution, accConstraints2, accConstraints1, 1, edge);


      } else {

        if (m_debug) std::cout << "TEST 4b" << std::endl;
        addConstraints(errorTime, handle2, handle1, criticalState2, solution, accConstraints2, accConstraints2, 0, edge);

      }

    }


    resultConstraints[handle1] = accConstraints1;
    resultConstraints[handle2] = accConstraints2;

  }



  bool checkHeadError(const State& errorState1, const State& errorState2, const size_t handle1, const size_t handle2, const auto& solution, auto& resultConstraints) {

    // If this is true, it is not an head error, it is an rear-end collision
    if (errorState1.dir == errorState2.dir) return false;

    // If there is no prev position, then Agent 2 try to get trough the initial state
    std::pair<State, int> resultAgent1 = getPrevPosition(errorState1, handle1, solution);
    if (resultAgent1.second == -1) return false;

    std::pair<State, int> resultAgent2 = getNextPosition(errorState1, handle2, solution);
    if (resultAgent2.second == -1) return false;

    if (resultAgent1.first.y != resultAgent2.first.y || resultAgent1.first.x != resultAgent2.first.x) {
      std::cout << "INFO: Head Error! Agent: "<< handle1 << ", Agent: " << handle2 << ", at: " << errorState1 << std::endl;

      Constraints accConstraints1;
      setConstrains(errorState1, getPrevPosition(errorState1, handle2, solution).second + 1, getNextPosition(errorState1, handle2, solution).second - 1, handle1, accConstraints1);
      resultConstraints[handle1] = accConstraints1;

      return true;
    }

    return false;
  }


  bool getFirstConflict(const auto& solution, auto& resultConstraints) {
    
    // Calculate the max time
    int max_t = 0;
    for (const auto& sol : solution) {
      max_t = std::max<int>(max_t, sol.states.size() - 1);
    }


    for (int t = 0; t < max_t; ++t) {
      // check drive-drive vertex collisions
      for (size_t handle1 = 0; handle1 < solution.size(); ++handle1) {
        if ((unsigned)t >= solution[handle1].states.size()) continue;

        State stateInizial1 = getState(handle1, solution, t);


        for (size_t handle2 = handle1 + 1; handle2 < solution.size(); ++handle2) {
          if ((unsigned)t >= solution[handle2].states.size()) continue;

          State stateInizial2 = getState(handle2, solution, t);


          if (stateInizial1.equalExceptTime(stateInizial2)) {

            // Check at first if there is an head error and then constrain agent 1 so he has to wait
            if (!checkHeadError(stateInizial1, stateInizial2, handle1, handle2, solution, resultConstraints)) {

              handleConflicts(stateInizial1, stateInizial2, t, handle1, handle2, solution, resultConstraints, 0);

            } 

            return true;
          }
        }
      }


      // drive-drive edge (swap)
      for (size_t handle1 = 0; handle1 < solution.size(); ++handle1) {
        if ((unsigned)t >= solution[handle1].states.size()) continue;

        State state1a = getState(handle1, solution, t);
        State state1b = getState(handle1, solution, t + 1);

        for (size_t handle2 = handle1 + 1; handle2 < solution.size(); ++handle2) {
          if ((unsigned)t >= solution[handle2].states.size()) continue;

          State state2a = getState(handle2, solution, t);
          State state2b = getState(handle2, solution, t + 1);


          // If there is a Edge Conflict
          if (state1a.equalExceptTime(state2b) && state1b.equalExceptTime(state2a)) {

            handleConflicts(state1b, state2b, t+1, handle1, handle2, solution, resultConstraints, 1);

            return true;
          }

        }
      }
    }

    return false;
  }


  std::pair<State, int> getNextPosition(const State& state, const int handle, const auto& solution) {

    // Start at the State in the next timestep and loop trough it, until there was another position
    for (int currTime = state.time+1; (unsigned)currTime < solution[handle].states.size(); currTime++) {

      State currState = solution[handle].states[currTime].first;

      if (state.y != currState.y || state.x != currState.x) return std::make_pair(currState, currTime);
    }

    // Return the goal, because the given State was already the goal
    return std::make_pair(solution[handle].states.back().first, -1);
  }


  std::pair<State, int> getPrevPosition(const int stateTime, const int handle, const auto& solution) {

    const State state = solution[handle].states[stateTime].first;

    for (int currTime = state.time-1; currTime >= 0; currTime--) {

      State currState = solution[handle].states[currTime].first;

      if (state.y != currState.y || state.x != currState.x) return std::make_pair(currState, currTime);
    }

    // Return the first (initial State) because we were already in this state
    return std::make_pair(solution[handle].states[0].first, -1);
  }


  std::pair<State, int> getPrevPosition(const State& state, const int handle, const auto& solution) {

    for (int currTime = state.time-1; currTime >= 0; currTime--) {

      State currState = solution[handle].states[currTime].first;

      if (state.y != currState.y || state.x != currState.x) return std::make_pair(currState, currTime);
    }

    // Return the first (initial State) because we were already in this state
    return std::make_pair(solution[handle].states[0].first, -1);
  }


  void setConstrains (const State& constrainedState, const int startingTime, const int endingTime, const size_t handle, Constraints& cons) {

    if (true) std::cout << "Constrain Agent: " << handle << " from: " << startingTime << ", to: " << endingTime << ", at: " << constrainedState << std::endl;


    for (int timeVertex = startingTime; timeVertex <= endingTime; timeVertex++) {

      cons.vertexConstraints.emplace(VertexConstraint(timeVertex, constrainedState.y, constrainedState.x));
      // if (m_debug) std::cout << "Agent " << handle << ": " << VertexConstraint(timeVertex, constrainedState.y, constrainedState.x) << std::endl;

    }
  }


  void calculateUniqueSolution(const int startTime, const auto& solution, auto& uniqueSolution, bool reverse=false) {
    // Assuming these have the same size and every state corresponds to the action
    const auto states = solution.states;
    const auto actions = solution.actions;


    // Make sure to get the initial state in the unique solution
    uniqueSolution.emplace_back(states[startTime].first);

    if (m_debug) std::cout << states[startTime].first << std::endl;

    if (!reverse)

      for (int time = startTime+1; (unsigned) time < states.size(); time++) {

        if (actions[time].first == Action::Wait) continue;

        uniqueSolution.emplace_back(states[time].first);
        if (m_debug) std::cout << states[time].first << std::endl;

    }

    else {

      for (int time = startTime-1; time >= 0; time--) {

        if (actions[time].first == Action::Wait) continue;

        uniqueSolution.emplace_back(states[time].first);
        if (m_debug) std::cout << states[time].first << std::endl;

      }

    }
  }


  bool isGoalOfAgent(const State s, const int handle) {
    return s.x == m_goals[handle].x && s.y == m_goals[handle].y;
  }

  int calculateJunctionHeuristics(const int handle, const State& criticalState, const State& nextSolutionState) {

    const auto& next_actions = m_edges[Location(criticalState.y, criticalState.x, criticalState.dir)];

    std::cout << criticalState << std::endl;

    for (const auto& na : next_actions) {

      std::cout << na << ", value: " << m_map.getHeuristicValue(handle, na.y, na.x, na.new_dir) << std::endl;
      if (nextSolutionState.y == na.y && nextSolutionState.x == na.x) continue;
      
      return m_map.getHeuristicValue(handle, na.y, na.x, na.new_dir);
    }

    // If there is no action, which has other coordinates than the current solution, there is no way out of this situation
    return -1;
  }

  void handleApplicableJunction(const int errorTime, const State& criticalState, const size_t handle1, const size_t handle2, const int edge, auto& cons, const auto& solution) {

    // Agent 1 collided at errorState with the other Agent. So Agent 1 is not allowed to get in this cell from the time
    // they collided until the time, both agents has for the first time another solution 


    if (isGoalOfAgent(criticalState, handle1)) {
      std::cout << "ACHTUNG" << handle1 << std::endl;
      // assert(0);
    }

    if (isGoalOfAgent(criticalState, handle2)) {
      std::cout << "ACHTUNG" << handle2 << std::endl;
      // assert(0);
    }


    std::vector<State> uniqueSolutionHandle1;
    std::vector<State> uniqueSolutionHandle2;

    if (m_debug) std::cout << "UniqueSolution (reverse) handle " << handle1 << std::endl;
    calculateUniqueSolution(errorTime, solution[handle1], uniqueSolutionHandle1, true);

    if (m_debug) std::cout << "UniqueSolution handle " << handle2 << std::endl;
    calculateUniqueSolution(errorTime - edge, solution[handle2], uniqueSolutionHandle2);

    const size_t sizeUniqueSolutionHandle1 = uniqueSolutionHandle1.size();
    const size_t sizeUniqueSolutionHandle2 = uniqueSolutionHandle2.size();



    // At the error time there is the same position, so we can start with the next state
    
    int endingTime = -1;
    int stepTime;
    int stepCriticalStateHandle2 = -1;

    for (stepTime = 0; (unsigned) stepTime < std::min(sizeUniqueSolutionHandle1, sizeUniqueSolutionHandle2); stepTime++) {

      State state1 = uniqueSolutionHandle1[stepTime];
      State state2 = uniqueSolutionHandle2[stepTime];


      if (state2.y == criticalState.y && state2.x == criticalState.x ) stepCriticalStateHandle2 = stepTime;

      // The time where agent 2 has for the first time another position
      if (state1.y != state2.y || state1.x != state2.x) endingTime = state2.time;

    }


    // If the endlingTime is still -1 then, the solution of both agents are identical 
    if (endingTime == -1) {

      // If the size of the Solution of Agent 1 is bigger, then Agent 2 drove into the Goal, so constrain to the time of the last state of the solution of agent 2
      if (sizeUniqueSolutionHandle1 > sizeUniqueSolutionHandle2) endingTime = solution[handle2].states.back().first.time; 

      else {

        const State nextSolutionState = getNextPosition(criticalState, handle1, solution).first;

        const int resInitialHeuristics =  calculateJunctionHeuristics(handle1, criticalState, nextSolutionState);

        // There is no way to come out of this situation, so return that this node can't get expanded further
        if (resInitialHeuristics == -1) return;

        std::cout << "Heuristic Constraints:" << std:: endl;
        std::cout << "Critical State: " << criticalState << std::endl;

        setConstrains(criticalState, criticalState.time+1, resInitialHeuristics+1, handle1, cons);

        setConstrains(nextSolutionState, criticalState.time+1, resInitialHeuristics+1, handle1, cons);

        return;
      }

    }


    // Make sure to add the steps, the agent needs to take from the critical point to the position they first separate and add it to
    // the constraints, so if the agent stay, in the next iteration he will start waiting and the other applicable route is not better
    else endingTime += criticalState.time - uniqueSolutionHandle1[stepTime-1].time;



    if (errorTime == criticalState.time) {

      std::pair<State, int> resPrevHandle2 = getPrevPosition(errorTime - edge, handle2, solution);

      if (resPrevHandle2.second == -1) return;  // Not very likely to trigger

      setConstrains(criticalState, resPrevHandle2.second+1, endingTime, handle1, cons);
    }


    // Make sure to block the path at the critical cell, so it is easier in the next iteration to detect 
    else setConstrains(criticalState, criticalState.time+1, endingTime, handle1, cons);


    const int startingTime = criticalState.time+1;

    // If not then we just constrain the cell after the critical state (make sure to pay attention for initial state

    std::pair<State, int> resNextHandle1 = getNextPosition(criticalState, handle1, solution);

    if (resNextHandle1.second == -1) return;  // Then the criticalState is the goal and errorTime == criticalState.time, where we have set the constraints already

    setConstrains(resNextHandle1.first, startingTime, endingTime, handle1, cons);

  }


  void handleNotApplicableJunction(const int errorTime, const State& criticalState, const size_t handle1, const size_t handle2, const int edge, auto& cons, const auto& solution) {


    const auto solutionHandle1 = solution[handle1];
    const auto solutionHandle2 = solution[handle2];

    std::vector<State> uniqueSolutionHandle1;
    std::vector<State> uniqueSolutionHandle2;

    if (m_debug) std::cout << "UniqueSolution (reverse) handle " << handle1 << std::endl;
    calculateUniqueSolution(errorTime, solutionHandle1, uniqueSolutionHandle1, true);

    if (m_debug) std::cout << "UniqueSolution handle " << handle2 << std::endl;
    calculateUniqueSolution(errorTime - edge, solutionHandle2, uniqueSolutionHandle2);

    const size_t sizeUniqueSolutionHandle1 = uniqueSolutionHandle1.size();
    const size_t sizeUniqueSolutionHandle2 = uniqueSolutionHandle2.size();


    // At the error time there is the same position, so we can start with the next state
    
    int endingTime = -1;
    int stepTime;

    int stepCriticalStateHandle2 = -1;

    for (stepTime = 0; (unsigned) stepTime < std::min(sizeUniqueSolutionHandle1, sizeUniqueSolutionHandle2); stepTime++) {

      State state1 = uniqueSolutionHandle1[stepTime];
      State state2 = uniqueSolutionHandle2[stepTime];

      // Get the step of the critical state of Agent 2, so we can use it later
      if (state2.y == criticalState.y && state2.x == criticalState.x ) stepCriticalStateHandle2 = stepTime;

      // The time where agent 2 has for the first time another position
      if (state1.y != state2.y || state1.x != state2.x) endingTime = state2.time;

    }


    if (errorTime == criticalState.time) {

      std::pair<State, int> resPrevHandle1Init = getPrevPosition(errorTime, handle1, solution);

      if (resPrevHandle1Init.second == -1) return;

      // Where Agent2 got into the cell until the error time
      setConstrains(criticalState, resPrevHandle1Init.second+1, errorTime, handle1, cons);
    }
  
    if (errorTime == criticalState.time && edge == 1) return;


    if (endingTime == -1) {

      // The Agent 1 needs to wait in front of the Goal to let the other agent trough

      // Passt
      if (sizeUniqueSolutionHandle1 > sizeUniqueSolutionHandle2) endingTime = solution[handle2].states.back().first.time; 

      // The other Agent found no other way but now he have to 
      else {

        // When there is no state after the critical state (Goal)
        if ((unsigned)stepCriticalStateHandle2+1 < sizeUniqueSolutionHandle2) {

          endingTime = uniqueSolutionHandle2[stepCriticalStateHandle2+1].time-1;

        }

      }
    }

    std::pair<State, int> resPrevHandle1 = getPrevPosition(criticalState, handle1, solution);

    // Should never trigger
    if (resPrevHandle1.second == -1) return;

    const int startingTime = resPrevHandle1.second+1;

    setConstrains(criticalState, startingTime, endingTime, handle1, cons);
  }


  void onExpandHighLevelNode(int /*cost*/) { m_highLevelExpanded++; }

  void onExpandLowLevelNode(const State& /*s*/, int /*fScore*/, int /*gScore*/) {m_lowLevelExpanded++;}

  int highLevelExpanded() { return m_highLevelExpanded; }

  int lowLevelExpanded() const { return m_lowLevelExpanded; }

 private:
  
  State getState(size_t agentIdx, const std::vector<PlanResult<State, Action, int> >& solution, size_t t) {
    return solution[agentIdx].states[t].first;
  }

  bool stateValid(const State& s) {
    assert(m_constraints);
    const auto& con = m_constraints->vertexConstraints;

    return con.find(VertexConstraint(s.time, s.y, s.x)) == con.end();;
  }

 private:
  int m_dimy;
  int m_dimx;
  std::unordered_map<Location, std::vector<NewLocation>> m_edges;
  std::vector<Location> m_goals;
  Map m_map;
  std::vector<Location> m_critical;
  size_t m_agentIdx;
  const Constraints* m_constraints;
  int m_lastGoalConstraint;
  int m_highLevelExpanded;
  int m_lowLevelExpanded;
  bool m_debug = true;
};



int main(int argc, char* argv[]) {
  namespace po = boost::program_options;
  // Declare the supported options.
  po::options_description desc("Allowed options");
  std::string inputFile;
  std::string inputGridFile;
  std::string outputFile;
  std::string replanFile;

  desc.add_options()("help", "produce help message")
    ("input,i", po::value<std::string>(&inputFile)->required(), "input file (YAML)")
    ("inputGrid,g", po::value<std::string>(&inputGridFile)->required(), "inputGrid file (txt)")
    ("output,o", po::value<std::string>(&outputFile)->required(), "output file (YAML)")
    ("replan,r", po::value<std::string>(&replanFile)->default_value(""), "the informations based on the replanning file (YAML)");

  try {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help") != 0u) {
      std::cout << desc << "\n";
      return 0;
    }
  } catch (po::error& e) {
    std::cerr << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }



  std::vector<State> startStates;
  std::vector<Location> goals;

  std::vector<Constraints> replanConstraints;
  // std::vector<PlanResult<State, Action, int>> replanResults;
  // int replanCost;


  // If there is a file provided, then use this file as an the starting HighLevelNode
  if (replanFile != "") {

    YAML::Node highLevelNode = YAML::LoadFile(replanFile);

    // replanCost = highLevelNode["cost"].as<int>();

    const auto& agents = highLevelNode["agents"];

    for (const auto& a : agents) {

      startStates.emplace_back(State(0, a["startState"]["y"].as<int>(), a["startState"]["x"].as<int>(), a["startState"]["dir"].as<int>()));
      
      Constraints constraints;
      for (const auto& con : a["constraints"]) {
        constraints.vertexConstraints.emplace(VertexConstraint(con["t"].as<int>(), con["y"].as<int>(), con["x"].as<int>()));
      }


      replanConstraints.emplace_back(constraints);
    }
  }


  // Loading and initializing the Map with its grind and the grid actions (TODO)
  Map map(inputFile, inputGridFile);
  map.inizialize();


  YAML::Node config = YAML::LoadFile(inputFile);

  std::unordered_map<Location, std::vector<NewLocation>> edges;
  std::vector<Location> critical;


  // Make sure that there is dimy at first and then there is dimx
  const auto& dim = config["map"]["dimensions"];
  int dimy = dim[0].as<int>();
  int dimx = dim[1].as<int>();

  // Insert the critical cells into the vector
  for (const auto& crit : config["map"]["critical"]) {
    critical.emplace_back(Location(crit[0].as<int>(), crit[1].as<int>()));
  }


  for (const auto& node : config["map"]["edges"]) {
    const auto& from = node["from"];
    const auto& direction = node["direction"];
    
    const auto& to = node["to"];
    const auto& new_dir = node["new_direction"];
    const auto& action = node["action"];

    edges[Location(from[0].as<int>(), from[1].as<int>(), direction.as<int>())].emplace_back(NewLocation(to[0].as<int>(), to[1].as<int>(), new_dir.as<int>(), action.as<int>()));
  }

  // Make sure to initialize all the agents with its starting values
  for (const auto& node : config["agents"]) {
    

    if (replanFile == "") {
      const auto& start = node["start"];
      const auto& direction = node["direction"];
      startStates.emplace_back(State(0, start[0].as<int>(), start[1].as<int>(), direction.as<int>()));
    }

    const auto& goal = node["goal"];
    
    goals.emplace_back(Location(goal[0].as<int>(), goal[1].as<int>()));
  }

  // Create the Environment based on the collected parameters and search with CBS
  Environment mapf(dimy, dimx, edges, goals, map, critical);
  CBS<State, Action, int, Conflict, Constraints, Environment> cbs(mapf);

  bool success;

  std::vector<PlanResult<State, Action, int> > solution;

  Timer timer;
  if (replanFile != "") {
    success = cbs.search(startStates, solution, replanConstraints, true);
  }

  else {
    // replanConstraints is only that the other function has no problems when compiling
    success = cbs.search(startStates, solution, replanConstraints);
  }
  timer.stop();

  if (success) {
    std::cout << "Planning Successful! " << std::endl;
    int cost = 0;
    int makespan = 0;
    for (const auto& s : solution) {
      cost += s.cost;
      makespan = std::max<int>(makespan, s.cost);
    }

    std::ofstream out(outputFile);
    out << "statistics:" << std::endl;
    out << "  cost: " << cost << std::endl;
    out << "  makespan: " << makespan << std::endl;
    out << "  runtime: " << timer.elapsedSeconds() << std::endl;
    out << "  highLevelExpanded: " << mapf.highLevelExpanded() << std::endl;
    out << "  lowLevelExpanded: " << mapf.lowLevelExpanded() << std::endl;
    out << "schedule:" << std::endl;
    for (size_t a = 0; a < solution.size(); ++a) {

      // Only for printing the output of the agents in a nice way to the terminal
      std::cout << "Solution for: " << a << std::endl;
      for (size_t i = 0; i < solution[a].actions.size(); ++i) {
        std::cout << solution[a].states[i].second << ": " <<
        solution[a].states[i].first << "->" << solution[a].actions[i].first
        << "(cost: " << solution[a].actions[i].second << ")" << std::endl;
      }
      std::cout << solution[a].states.back().second << ": " <<
      solution[a].states.back().first << std::endl;


      // Creating the output by writing all actions of the certain agent into the file
      out << "  agent" << a << ":" << std::endl;
      for (const auto& state : solution[a].actions) {
        out << "    - " << state.first << std::endl;
      }


      // Use this for visualizing in the python tool
      /*
      out << "  agent" << a << ":" << std::endl;
      for (const auto& state : solution[a].states) {
        out << "    - x: " << state.first.x << std::endl
            << "      y: " << state.first.y << std::endl
            << "      t: " << state.second << std::endl;
      }
      */

      
    }
  } else {
    std::cout << "Planning NOT successful!" << std::endl;
  }

  return 0;
}
