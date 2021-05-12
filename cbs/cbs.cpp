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
  VertexConstraint(int time, int y, int x, int setFrom=-1, int otherSetFrom=-1) : time(time), y(y), x(x), setFrom(setFrom), otherSetFrom(otherSetFrom) {}
  int time;
  int y;
  int x;
  int setFrom;
  int otherSetFrom;

  bool operator<(const VertexConstraint& other) const {
    return std::tie(time, x, y) < std::tie(other.time, other.x, other.y);
  }

  bool operator==(const VertexConstraint& other) const {
    return std::tie(time, x, y) == std::tie(other.time, other.x, other.y);
  }

  friend std::ostream& operator<<(std::ostream& os, const VertexConstraint& c) {
    if (c.setFrom == -1 && c.otherSetFrom != -1) return os << "VC(t=" << c.time << ",y=" << c.y << ",x=" << c.x << ",osf=" << c.otherSetFrom << ")"; 

    else if (c.setFrom != -1 && c.otherSetFrom == -1) return os << "VC(t=" << c.time << ",y=" << c.y << ",x=" << c.x << ",sf=" << c.setFrom << ")"; 

    else return os << "VC(t=" << c.time << ",y=" << c.y << ",x=" << c.x << ")";

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

  void del(const Constraints& other) {

    for (const auto& c : other.vertexConstraints) {
      vertexConstraints.erase(c);
    }

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
              std::vector<Location> goals, Map map, std::vector<Location> critical, std::vector<int> speeds,
              std::vector<Constraints>& startingConstraints, bool replanning)
      : m_dimy(dimy),
        m_dimx(dimx),
        m_edges(std::move(edges)),
        m_goals(std::move(goals)),
        m_map(map),
        m_critical(critical),
        m_speeds(speeds),
        m_startingConstraints(startingConstraints),
        m_replanning(replanning),

        // Local variables
        m_agentIdx(0),
        m_constraints(nullptr),
        m_lastGoalConstraint(-1),
        m_highLevelExpanded(0),
        m_lowLevelExpanded(0) {
  }

  Environment(const Environment&) = delete;
  Environment& operator=(const Environment&) = delete;

  void setLowLevelContext(size_t agentIdx, int speed, int replannigSize, const Constraints* constraints) {
    assert(constraints);  // NOLINT
    m_agentIdx = agentIdx;
    m_agentSpeed = speed;
    m_agentReplanningSize = replannigSize;
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

  void getNeighbors(const State& s, std::vector<Neighbor<State, Action, int> >& neighbors) {

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

  int getDirectionCritical(int errorTime, const State searchingState, const size_t handle, const auto& solution) {

  
    for (int t = errorTime; (unsigned)t < solution[handle].states.size(); t++) {

      State checkState = getState(handle, solution, t);

      if (searchingState.y == checkState.y && searchingState.x == checkState.x) return checkState.dir;
    }

    return -1;
  }

  State getCriticalState(int t, size_t handle1, size_t handle2, const auto& solution) {

    int t_back;

    for (t_back = t; t_back >= 0; t_back--) {

      State checkState = getState(handle1, solution, t_back);
      
      if (!checkCriticalLocation(checkState)) {
        continue;  // The current state position is not a critical Point
      }

      // If the critical Point is the goal of the other agent, then return this one
      if (checkState.x == m_goals[handle2].x && checkState.y == m_goals[handle2].y && t_back > 0) {
        return checkState;
      }

      // If it is a critical state, then check if minimum one of the agents can use it
      if (m_edges[Location(checkState.y, checkState.x, checkState.dir)].size() >= 2) {
        return checkState;
      }

      int dir = getDirectionCritical(t, checkState, handle2, solution);
      if (dir == -1) continue;

      if (m_edges[Location(checkState.y, checkState.x, dir)].size() >= 2) {
        return checkState;  
      }

      // If there was no return statement and both of the agents can't use the critical point, go to the other critical point

    }

    return State(-1, -1, -1, -1);
  }


  void addConstraints(int errorTime, size_t handle1, size_t handle2, State& criticalState, auto& solution, auto& resultConstraints, auto& resultDoubleConstraints, auto& resultDeleteConstraints, int type, int edge, auto& conflicts) {

    Constraints accConstraints1;
    Constraints accConstraints2;

    bool deleteConstraints;

    if (type == 1) handleApplicableJunction(errorTime, criticalState, handle1, handle2, edge, accConstraints1, accConstraints2, solution);


    else if (type == 0) deleteConstraints = handleNotApplicableJunction(errorTime, criticalState, handle1, handle2, edge, accConstraints1, accConstraints2, solution, conflicts);


    if (deleteConstraints) {

      resultDeleteConstraints[std::make_pair(handle1, handle2)] = std::make_pair(accConstraints1, accConstraints2);
      return;
    }


    // If both of the agents got constrained
    if (accConstraints1.vertexConstraints.size() > 0 && accConstraints2.vertexConstraints.size() > 0) resultDoubleConstraints[std::make_pair(handle1, handle2)] = std::make_pair(accConstraints1, accConstraints2);

    else if (accConstraints1.vertexConstraints.size() > 0) resultConstraints.emplace_back(std::make_pair(handle1, accConstraints1));

    else if (accConstraints2.vertexConstraints.size() > 0) resultConstraints.emplace_back(std::make_pair(handle2, accConstraints2));


    return;
  }


  void handleConflicts(const State& errorState1, const State& errorState2, const int errorTime, const size_t handle1, const size_t handle2, const auto& solution, auto& resultConstraints, auto& resultDoubleConstraints, auto& resultDeleteConstraints, int edge, auto& conflicts) {


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


    // Check if the the error states of the Agents is a Vertex error and if they have the same direction, then one have to wait for the other
    if (errorState1.dir == errorState2.dir && edge == 0) {

      
      

      if (m_debug) std::cout << "Aufgefahren!" << std::endl;

      /*
      std::cout << "Solution Handle: " << handle1 << std::endl;

      for (const auto& c : solution[handle1].states) {
        std::cout << c.first << std::endl;
      }

      std::cout << "Solution Handle: " << handle2 << std::endl;

      for (const auto& c : solution[handle2].states) {
        std::cout << c.first << std::endl;
      }
      */


      // Agent1 is waiting for another so, the previous agent has to wait
      if (errorState1.equalExceptTime(solution[handle1].states[errorState1.time-1].first)) {
        
        Constraints accConstraints2;

        // Loop troth the vertex error time and check how long the agent has to wait and add the constraints to the other agents
        for (int waitTime = errorTime; (unsigned)waitTime < solution[handle1].states.size(); waitTime++) {

          if (!solution[handle1].states[waitTime].first.equalExceptTime(errorState2)) break;

          accConstraints2.vertexConstraints.insert(VertexConstraint(waitTime, errorState1.y, errorState1.x, handle1, -1));
        }

        resultConstraints.emplace_back(std::make_pair(handle2, accConstraints2));
      }

              
      else if (errorState2.equalExceptTime(solution[handle2].states[errorState2.time-1].first)) {

        Constraints accConstraints1;

        for (int waitTime = errorTime; (unsigned)waitTime < solution[handle2].states.size(); waitTime++) {

          if (!solution[handle2].states[waitTime].first.equalExceptTime(errorState1)) break;

          accConstraints1.vertexConstraints.insert(VertexConstraint(waitTime, errorState2.y, errorState2.x, handle2, -1));
        }

        resultConstraints.emplace_back(std::make_pair(handle1, accConstraints1));
      }

      return;
    }


    // There is no way the agent can get over each other, so return true and don't expand this HighLevelNode further
    else if (criticalState1.time == -1 && criticalState2.time == -1) {
      if (m_debug) std::cout << "Agent: "<<  handle1 << " & Agent: " <<  handle2 << " can't pass each other!" << std::endl;
      return;
    }

    // Agent 1 cant get out of the situation 
    else if (criticalState1.time == -1) {

      addConstraints(errorTime, handle2, handle1, criticalState2, solution, resultConstraints, resultDoubleConstraints, resultDeleteConstraints, m_edges[Location(criticalState2.y, criticalState2.x, criticalState2.dir)].size() >= 2, edge, conflicts);

    }

    // Agent 2 cant get out of the situation 
    else if (criticalState2.time == -1) {
      
      addConstraints(errorTime, handle1, handle2, criticalState1, solution, resultConstraints, resultDoubleConstraints, resultDeleteConstraints, m_edges[Location(criticalState1.y, criticalState1.x, criticalState1.dir)].size() >= 2, edge, conflicts);

    } 

    // Both Agents can get out of the situation
    else {  

      // Constrain Agent 1
      addConstraints(errorTime, handle1, handle2, criticalState1, solution, resultConstraints, resultDoubleConstraints, resultDeleteConstraints, m_edges[Location(criticalState1.y, criticalState1.x, criticalState1.dir)].size() >= 2, edge, conflicts);

      // Constrain Agent 2
      addConstraints(errorTime, handle2, handle1, criticalState2, solution, resultConstraints, resultDoubleConstraints, resultDeleteConstraints, m_edges[Location(criticalState2.y, criticalState2.x, criticalState2.dir)].size() >= 2, edge, conflicts);

    }
  }


  bool checkHeadError(const State& errorState1, const State& errorState2, const size_t handle1, const size_t handle2, const auto& solution, auto& resultConstraints) {

    // If this is true, it is not an head error, it is an rear-end collision
    if (errorState1.dir == errorState2.dir) return false;

    if (m_debug) std::cout << "Check for Head" << std::endl << errorState1 << std::endl;

    // If there is no prev state, then the other agent want to go trough the starting state
    std::pair<State, int> resultAgent1 = getPrevPosition(errorState1, handle1, solution);
    if (m_debug) std::cout << "Prev: " << resultAgent1.first << std::endl;

    std::pair<State, int> resultAgent2 = getNextPosition(errorState2, handle2, solution);
    if (m_debug) std::cout << "Next: " << resultAgent2.first << std::endl;
    
    if (resultAgent1.second == -1) {

      // If there is no Prev state, then both of the agents started on the same spot
      std::pair<State, int> resultPrevAgent2 = getPrevPosition(errorState2, handle2, solution);
      if (resultPrevAgent2.second == -1) return false;


      // The time Agent 2 would have entered the cell of the agent 1
      const int startingTime = resultPrevAgent2.second+1;

      // The time Agent 1 would leave the cell
      // If there is no next state, then the goal is the starting point (not possible)
      const int endingTime   = getNextPosition(errorState1, handle1, solution).second-1;

      Constraints accConstraints2;
      setConstrains(errorState1, startingTime, endingTime, handle2, accConstraints2);
      resultConstraints.emplace_back(std::make_pair(handle2, accConstraints2));


      return true;
    }

    
    if (resultAgent2.second == -1) return false;

    
    

    if (resultAgent1.first.y != resultAgent2.first.y || resultAgent1.first.x != resultAgent2.first.x) {
      if (m_debug) std::cout << "INFO: Head Error! Agent: "<< handle1 << ", Agent: " << handle2 << ", at: " << errorState1 << std::endl;

      Constraints accConstraints1;
      setConstrains(errorState1, getPrevPosition(errorState1, handle1, solution).second + 1, getNextPosition(errorState2, handle2, solution).second - 1, handle1, accConstraints1);
      resultConstraints.emplace_back(std::make_pair(handle1, accConstraints1));

      return true;
    }

    return false;
  }


  bool getFirstConflict(const auto& solution, auto& resultConstraints, auto& resultDoubleConstraints, auto& resultDeleteConstraints, auto& conflicts) {

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

              handleConflicts(stateInizial1, stateInizial2, t, handle1, handle2, solution, resultConstraints, resultDoubleConstraints, resultDeleteConstraints, 0, conflicts);

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

            handleConflicts(state1b, state2b, t+1, handle1, handle2, solution, resultConstraints, resultDoubleConstraints, resultDeleteConstraints, 1, conflicts);

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


  void setConstrains (const State& constrainedState, const int startingTime, const int endingTime, const size_t handle, Constraints& cons, const size_t handleOther=-1, const bool otherSetFrom = false) {

    if (m_debug) std::cout << "Constrain Agent: " << handle << " from: " << startingTime << ", to: " << endingTime << ", at: " << constrainedState << std::endl;


    for (int timeVertex = startingTime; timeVertex <= endingTime; timeVertex++) {

      if (otherSetFrom) cons.vertexConstraints.emplace(VertexConstraint(timeVertex, constrainedState.y, constrainedState.x, -1, handleOther));

      else cons.vertexConstraints.emplace(VertexConstraint(timeVertex, constrainedState.y, constrainedState.x, handleOther, -1));

    }
  }


  void calculateUniqueSolution(const int startTime, const auto& solution, auto& uniqueSolution, bool reverse=false) {
    // Assuming these have the same size and every state corresponds to the action
    const auto states = solution.states;
    const auto actions = solution.actions;


    // Make sure to get the initial state in the unique solution
    uniqueSolution.emplace_back(states[startTime].first);
    int acc = 0;

    if (m_debug) std::cout << states[startTime].first << std::endl;

    if (!reverse)

      for (int time = startTime+1; (unsigned) time < states.size(); time++) {

        State currState = states[time].first;

        if (uniqueSolution[acc].y == currState.y && uniqueSolution[acc].x == currState.x) continue;

        uniqueSolution.emplace_back(currState);
        acc++;

        if (m_debug) std::cout << currState << std::endl;

    }

    else {

      for (int time = startTime-1; time >= 0; time--) {

        State currState = states[time].first;

        if (uniqueSolution[acc].y == currState.y && uniqueSolution[acc].x == currState.x) continue;

        uniqueSolution.emplace_back(currState);
        acc++;

        if (m_debug) std::cout << currState << std::endl;

      }

    }
  }


  bool isGoalOfAgent(const State s, const int handle) {
    return s.x == m_goals[handle].x && s.y == m_goals[handle].y;
  }

  int calculateJunctionHeuristics(const int handle, const State& criticalState, const State& nextSolutionState) {

    const auto& next_actions = m_edges[Location(criticalState.y, criticalState.x, criticalState.dir)];

    for (const auto& na : next_actions) {

      if (m_debug) std::cout << na << ", value: " << m_map.getHeuristicValue(handle, na.y, na.x, na.new_dir) << std::endl;
      if (nextSolutionState.y == na.y && nextSolutionState.x == na.x) continue;
      
      return m_map.getHeuristicValue(handle, na.y, na.x, na.new_dir);
    }

    // If there is no action, which has other coordinates than the current solution, there is no way out of this situation
    return -1;
  }

  void handleApplicableJunction(const int errorTime, const State& criticalState, const size_t handle1, const size_t handle2, const int edge, auto& cons, auto& consOther, const auto& solution) {

    if (m_debug) std::cout << std::endl << "Handle applicable:" << std::endl;

    std::vector<State> uniqueSolutionHandle1;
    std::vector<State> uniqueSolutionHandle2;

    if (m_debug) std::cout << "UniqueSolution (reverse) handle " << handle1 << std::endl;
    calculateUniqueSolution(errorTime, solution[handle1], uniqueSolutionHandle1, true);

    if (m_debug) std::cout << "UniqueSolution handle " << handle2 << std::endl;
    calculateUniqueSolution(errorTime - edge, solution[handle2], uniqueSolutionHandle2);

    const size_t sizeUniqueSolutionHandle1 = uniqueSolutionHandle1.size();
    const size_t sizeUniqueSolutionHandle2 = uniqueSolutionHandle2.size();


    
    int endingTime = -1;
    int stepTime;
    int stepCriticalState;

    for (stepTime = 0; (unsigned) stepTime < std::min(sizeUniqueSolutionHandle1, sizeUniqueSolutionHandle2); stepTime++) {

      State state1 = uniqueSolutionHandle1[stepTime];
      State state2 = uniqueSolutionHandle2[stepTime];

      // The other Agent needs to pass at least the critical state, so stepCriticalState is never 0
      if (state1.y == criticalState.y && state1.x == criticalState.x ) stepCriticalState = stepTime;

      // To this time, the other agent is the last time in the state, he should leave
      if (state1.y != state2.y || state1.x != state2.x) {
        endingTime = state2.time - 1;
        break;
      }

    }


    // Here to constrain the other Agent
    if (errorTime == criticalState.time) {

      std::pair<State, int> resPrevHandle2 = getPrevPosition(errorTime - edge, handle2, solution);

      if (resPrevHandle2.second == -1) return;

      // If +1 the time, agent 1 would enter the error cell
      std::pair<State, int> resPrevHandle1 = getPrevPosition(errorTime, handle1, solution);

      if (resPrevHandle1.second == -1) return;

      
      // From: Where Agent 2 normally got into the error cell
      // To:   The time of the error
      setConstrains(criticalState, resPrevHandle2.second+1, resPrevHandle1.second+m_speeds[handle1], handle2, consOther, handle1, true);

      // If the criticalState is the goal of Agent 1, then let Agent 1 drive into the goal and don't constrain further
      if (isGoalOfAgent(criticalState, handle1)) return;
    }


    bool isNoGoalConstraint = true;
    // If the endingTime is still -1 then, the solution of both agents are identical 
    if (endingTime == -1) {

      // Constrain the critical and the next cell for the time of the time of the last state form agent 2
      if (sizeUniqueSolutionHandle1 > sizeUniqueSolutionHandle2) {
        isNoGoalConstraint = false;
        endingTime = uniqueSolutionHandle2.back().time - 1;  // Lower cost with -1
      }

      else {

        const State nextSolutionState = getNextPosition(criticalState, handle1, solution).first;

        const int resInitialHeuristics =  calculateJunctionHeuristics(handle1, criticalState, nextSolutionState);

        // There is no way to come out of this situation, so return that this node can't get expanded further
        if (resInitialHeuristics == -1) return;

        if (m_debug) std::cout << "Heuristic Constraints:" << std:: endl;

        endingTime = resInitialHeuristics;

        // TODO: Look for the steps which get added
      }

    }

    // The count from the critical state to the position, where both agents has to last common point
    if (isNoGoalConstraint) endingTime += (stepTime - stepCriticalState - 1) * m_speeds[handle1];

    // Prev time, time the agent needs in the cell, +1 so it can escape
    const int startingTime = getPrevPosition(criticalState, handle1, solution).second + m_speeds[handle1] + 1;

    State nextSolutionState = getNextPosition(criticalState, handle1, solution).first;  // Already took care of in errorTime == criticalState.time

    // Block the cell after the criticalState, so Agent 1 won't take this path
    setConstrains(nextSolutionState, startingTime, endingTime, handle1, cons, handle2);

    // Block the cell of the critical cell, so if will get detected in the next iteration better
    setConstrains(criticalState    , startingTime, endingTime, handle1, cons, handle2);
  }


  bool handleNotApplicableJunction(const int errorTime, const State& criticalState, const size_t handle1, const size_t handle2, const int edge, auto& cons, auto& consOther, const auto& solution, auto& conflicts) {

    if (errorTime == criticalState.time) {
      
      // If this the criticalState is the goal of the Agent 2, then constrain until the end
      if (isGoalOfAgent(criticalState, handle2)) {

        // Agent 1 would enter the error goal cell
        std::pair<State, int> resPrevHandle1 = getPrevPosition(errorTime, handle1, solution);

        if (resPrevHandle1.second == -1) return false;

        const int endingTime = solution[handle2].states.back().first.time;

        setConstrains(criticalState, resPrevHandle1.second+1, endingTime, handle1, cons, handle2);
      }

      return false;
    }

    if (m_debug) std::cout << std::endl << "Handle not applicable:" << std::endl;

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

    int deleteSteps = 0;


    // At the error time there is the same position, so we can start with the next state
    
    int stepTime;
    int endingTime = -1;
    int stepCriticalState;

    for (stepTime = 0; (unsigned) stepTime < std::min(sizeUniqueSolutionHandle1, sizeUniqueSolutionHandle2); stepTime++) {

      State state1 = uniqueSolutionHandle1[stepTime];
      State state2 = uniqueSolutionHandle2[stepTime];

      // Get the step of the critical state of Agent 2, so we can use it later
      if (state2.y == criticalState.y && state2.x == criticalState.x ) stepCriticalState = stepTime;

      // The time where agent 2 has for the first time another position
      if (state1.y != state2.y || state1.x != state2.x) {
        
        endingTime = state2.time-1;
        break;

      }

    }


    if (endingTime == -1) {

      // The Agent 1 needs to wait in front of the Goal to let the other agent trough

      if (sizeUniqueSolutionHandle1 > sizeUniqueSolutionHandle2) endingTime = solution[handle2].states.back().first.time;


      // The other Agent found no other way but now he have to 
      else {

        // If true, the goal and the critical cell are the same. There is no way out in this situation
        if ((unsigned) stepCriticalState+1 == sizeUniqueSolutionHandle2) return false;

        // The critical state from Agent 2. IMPORTANT because of direction
        const State criticalStateHandle2 = uniqueSolutionHandle2[stepCriticalState];

        // The state after the critical state of Agent 2
        const State nextSolutionState = uniqueSolutionHandle2[stepCriticalState+1];

        const int resInitialHeuristics =  calculateJunctionHeuristics(handle2, criticalStateHandle2, nextSolutionState);

        // There is no way to come out of this situation, so return that this node can't get expanded further
        if (resInitialHeuristics == -1) return false;

        if (m_debug) std::cout << "Heuristic Constraints (not applicable):" << std:: endl;

        // Constrain the agent from from one timestep after entering the critical cell
        const int startingTimeHandle2 = getPrevPosition(criticalStateHandle2, handle2, solution).second;

        if (startingTimeHandle2 == -1) return false;

        setConstrains(criticalStateHandle2, startingTimeHandle2+2, resInitialHeuristics, handle2, consOther, handle1, true);

        setConstrains(nextSolutionState   , startingTimeHandle2+2, resInitialHeuristics, handle2, consOther, handle1, true);


        endingTime = startingTimeHandle2+2;
      }
    }

    else {

      for (const auto& c : conflicts[handle2].vertexConstraints) {
        if ((unsigned) c.otherSetFrom == handle1) {
          if (m_debug) std::cout << "Pruned: " << c << std::endl;
          consOther.vertexConstraints.emplace(c);
          deleteSteps++;
        }
      }      
    }



    std::pair<State, int> resPrevHandle1 = getPrevPosition(criticalState, handle1, solution);

    if (resPrevHandle1.second == -1) return false; // Should never trigger

    // The time, Agent 1 would normally enter the critical cell
    const int startingTime = resPrevHandle1.second+1;



    if (m_edges[Location(resPrevHandle1.first.y, resPrevHandle1.first.x, resPrevHandle1.first.dir)].size() >= 2 && (unsigned) stepCriticalState+1 < sizeUniqueSolutionHandle2) {

      setConstrains(criticalState, startingTime, uniqueSolutionHandle2[stepCriticalState+1].time-1, handle1, cons, handle2);

    }

    else setConstrains(criticalState, startingTime, endingTime-deleteSteps, handle1, cons, handle2);

    return deleteSteps > 0;
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
    

    // std::cout << m_speeds[m_agentIdx] << std::endl;
    if (m_replanning) {
      const auto& startCon = m_startingConstraints[m_agentIdx].vertexConstraints;

      if ((unsigned) s.time <= startCon.size()) return startCon.find(VertexConstraint(s.time, s.y, s.x)) == startCon.end();
    }

    const auto& con = m_constraints->vertexConstraints;

    bool isValid = true;
    int starting = (s.time - m_agentReplanningSize) * m_agentSpeed + m_agentReplanningSize;
    int ending = starting + m_agentSpeed;

    for (int time = starting; starting < ending; starting++) {

      if (con.find(VertexConstraint(time, s.y, s.x)) != con.end()) {
        isValid = false;
        break;
      }
    }

    return isValid;
  }

 private:
  int m_dimy;
  int m_dimx;
  std::unordered_map<Location, std::vector<NewLocation>> m_edges;
  std::vector<Location> m_goals;
  Map m_map;
  std::vector<Location> m_critical;
  std::vector<int> m_speeds;
  const std::vector<Constraints> m_startingConstraints;
  bool m_replanning;

  // Local variables
  size_t m_agentIdx;
  size_t m_agentSpeed;
  size_t m_agentReplanningSize;
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
  std::vector<int> replanNextAssignment;


  // If there is a file provided, then use this file as an the starting HighLevelNode
  if (replanFile != "") {

    YAML::Node highLevelNode = YAML::LoadFile(replanFile);

    const auto& agents = highLevelNode["agents"];

    for (const auto& a : agents) {

      startStates.emplace_back(State(a["startState"]["t"].as<int>(), a["startState"]["y"].as<int>(), a["startState"]["x"].as<int>(), a["startState"]["dir"].as<int>()));
      
      Constraints constraints;
      for (const auto& con : a["constraints"]) {
        constraints.vertexConstraints.emplace(VertexConstraint(con["t"].as<int>(), con["y"].as<int>(), con["x"].as<int>()));
        std::cout << " " << VertexConstraint(con["t"].as<int>(), con["y"].as<int>(), con["x"].as<int>()) << std::endl;
      }

      replanConstraints.emplace_back(constraints);

      replanNextAssignment.emplace_back(a["next_replanning"].as<int>());

      std::cout << "Next replanning: " << a["next_replanning"] << std::endl;
    }
  }


  // Loading and initializing the Map with its grind and the grid actions (TODO)
  Map map(inputFile, inputGridFile);
  map.inizialize();


  YAML::Node config = YAML::LoadFile(inputFile);

  std::unordered_map<Location, std::vector<NewLocation>> edges;
  std::vector<Location> critical;
  std::vector<int> speeds;


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
  for (const auto& agent : config["agents"]) {
    

    // If there is no replan file, then take the states and directions of the given file. When not there was already took care of
    if (replanFile == "") {
      const auto& start = agent["start"];
      const auto& direction = agent["direction"];
      startStates.emplace_back(State(0, start[0].as<int>(), start[1].as<int>(), direction.as<int>()));
    }

    const auto& goal = agent["goal"];
    goals.emplace_back(Location(goal[0].as<int>(), goal[1].as<int>()));


    // Also get the speed of the agents
    speeds.emplace_back(agent["speed"].as<int>());

  }

  // Create the Environment based on the collected parameters and search with CBS
  Environment mapf(dimy, dimx, edges, goals, map, critical, speeds, replanConstraints, replanFile != "");
  CBS<State, Action, int, Conflict, Constraints, Environment> cbs(mapf);

  bool success;

  std::vector<PlanResult<State, Action, int> > solution;

  Timer timer;

  success = cbs.search(startStates, speeds, solution, replanConstraints, replanNextAssignment, replanFile != "");

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

      if (true) {
        // Only for printing the output of the agents in a nice way to the terminal
        std::cout << "Solution for: " << a << std::endl;
        for (size_t i = 0; i < solution[a].actions.size(); ++i) {
          std::cout << solution[a].states[i].first << std::endl;
        }
        std::cout << std::string(30, '*') << std::endl;
      }


      // Creating the output by writing all actions of the certain agent into the file
      out << "  agent" << a << ":" << std::endl;
      for (const auto& state : solution[a].actions) {
        out << "    - " << state.first << std::endl;
      }

      
    }
  } else {
    std::cout << "Planning NOT successful!" << std::endl;
  }

  return 0;
}
