#include <fstream>
#include <iostream>
#include <sstream>
#include <cmath>
#include <algorithm>

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

      if (m_grid_data[value] == -1) {
        std::cout << "heuristik ist einfach nur dumm!!" << std::endl << std::endl;
      }

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

struct EdgeConstraint {
  EdgeConstraint(int time, int y1, int x1, int y2, int x2)
      : time(time), y1(y1), x1(x1), y2(y2), x2(x2) {}
  int time;
  int y1;
  int x1;
  int y2;
  int x2;

  bool operator<(const EdgeConstraint& other) const {
    return std::tie(time, x1, y1, x2, y2) <
           std::tie(other.time, other.x1, other.y1, other.x2, other.y2);
  }

  bool operator==(const EdgeConstraint& other) const {
    return std::tie(time, x1, y1, x2, y2) ==
           std::tie(other.time, other.x1, other.y1, other.x2, other.y2);
  }

  friend std::ostream& operator<<(std::ostream& os, const EdgeConstraint& c) {
    return os << "EC(" << c.time << "," << c.y1 << "," << c.x1 << "," << c.y2
              << "," << c.x2 << ")";
  }
};

namespace std {
template <>
struct hash<EdgeConstraint> {
  size_t operator()(const EdgeConstraint& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x1);
    boost::hash_combine(seed, s.y1);
    boost::hash_combine(seed, s.x2);
    boost::hash_combine(seed, s.y2);
    return seed;
  }
};
}  // namespace std

struct Constraints {
  std::unordered_set<VertexConstraint> vertexConstraints;
  std::unordered_set<EdgeConstraint> edgeConstraints;

  void add(const Constraints& other) {
    vertexConstraints.insert(other.vertexConstraints.begin(),
                             other.vertexConstraints.end());
    edgeConstraints.insert(other.edgeConstraints.begin(),
                           other.edgeConstraints.end());
  }

  bool overlap(const Constraints& other) const {
    for (const auto& vc : vertexConstraints) {
      if (other.vertexConstraints.count(vc) > 0) {
        return true;
      }
    }

    return false;
  }

  friend std::ostream& operator<<(std::ostream& os, const Constraints& c) {
    for (const auto& vc : c.vertexConstraints) {
      os << vc << std::endl;
    }
    for (const auto& ec : c.edgeConstraints) {
      os << ec << std::endl;
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
      return os << "(" << c.y << "," << c.x << ")";
    }

    return os << "(" << c.y << "," << c.x << "," << c.dir << ")";    
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
    if (s.x == m_goals[m_agentIdx].x && s.y == m_goals[m_agentIdx].y && s.time > m_lastGoalConstraint) {
      // std::cout << "Goal reached! Agent: "<< m_agentIdx << std::endl;
    }

    return s.x == m_goals[m_agentIdx].x && s.y == m_goals[m_agentIdx].y &&
           s.time > m_lastGoalConstraint;
  }

  void getNeighbors(const State& s, std::vector<Neighbor<State, Action, int> >& neighbors) {

    neighbors.clear();

    {
      // Always append the do_nothing (4) State if valid
      State n(s.time + 1, s.y, s.x, s.dir);

      if (stateValid(n) && transitionValid(s, n)) {
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


        if (stateValid(n) && transitionValid(s, n)) {

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


  bool isGoalOfAgent(const State s, const int handle) {
    return s.x == m_goals[handle].x && s.y == m_goals[handle].y;
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


  void setConstrains(const State& constrainedState, const int startingTime, const int endingTime, const size_t handle, Constraints& cons) {

    std::cout << "INFO VERTEX: Set A[" << handle << "]: " << "(y=" << constrainedState.y << ", x=" << constrainedState.x 
    << "), time: " << startingTime << " to " << endingTime << std::endl;


    for (int timeVertex = startingTime; timeVertex <= endingTime; timeVertex++) {

      cons.vertexConstraints.emplace(VertexConstraint(timeVertex, constrainedState.y, constrainedState.x));
      if (m_debug) std::cout << "Agent " << handle << ": " << VertexConstraint(timeVertex, constrainedState.y, constrainedState.x) << std::endl;

    }
  }


  // This functions returns a Pair with the next State and the timestep of this 
  std::pair<State, int> getNextPosition(const State& state, const int handle, const std::vector<PlanResult<State, Action, int>>& solution) {

    int currTime;

    // Start at the State in the next timestep and loop trough it, until there was another position
    for (currTime = state.time+1; (unsigned)currTime < solution[handle].states.size(); currTime++) {

      State currState = solution[handle].states[currTime].first;

      if (state.y != currState.y || state.x != currState.x) return std::make_pair(currState, currTime);
    }


    // std::cerr << "ACHTUNG: In getNextPosition() gab es keinen Nachvolgerstate!!" << std::endl;

    // Make sure to handle this exception in the code, because it was already the last position of the the solution
    return std::make_pair(solution[handle].states[currTime-1].first, -1);
  }


  std::pair<State, int> getPrevPosition(const State& state, const int handle, const std::vector<PlanResult<State, Action, int>>& solution) {

    for (int currTime = state.time-1; currTime >= 0; currTime--) {

      State currState = solution[handle].states[currTime].first;

      // std::cout << currState << std::endl;

      if (state.y != currState.y || state.x != currState.x) return std::make_pair(currState, currTime);
    }

    // std::cerr << "ACHTUNG: In getPrevPosition() gab es keine vorherige Position!!" << std::endl;

    return std::make_pair(solution[handle].states[0].first, -1);
  }


  void handleConflicts(const State& stateInizial1, const State& stateInizial2, const size_t handle1, const size_t handle2, const std::vector<PlanResult<State, Action, int>>& solution, std::multimap<size_t, Constraints>& resultConstraints, int edge) {

    // Only for Debug purposes
    if (edge == 0) {
      if (m_debug) std::cout << std::endl << "Vertex Error at: " << stateInizial1 << " and " << stateInizial2 << std::endl;
    } else {
      if (m_debug) std::cout << std::endl << "Edge Error at: " << stateInizial1 << " and " << stateInizial2 << std::endl;
    }



    State runningStateAgent1 = stateInizial1;
    State runningStateAgent2 = stateInizial2;

    State constrainedState = stateInizial1;  // hauptsache es ist was da
    int startingTime;
    int endingTime;


    if (edge == 1) runningStateAgent2 = getPrevPosition(stateInizial2, handle2, solution).first;

    std::cout << "INFO: Lsg: A[" << handle1 << "]: " << runningStateAgent1 << " <-> A[" << handle2 << "]: " << runningStateAgent2 << ";" << std::endl;

    Constraints accConstraints;

    if (stateInizial1.dir == stateInizial2.dir && edge == 0) {

      if (stateInizial1.equalExceptTime(solution[handle1].states[stateInizial1.time-1].first)) {
        int waitTime;

        // Loop troth the vertex error time and check how long the agent has to wait and add the constraints to the other agents
        for (waitTime = stateInizial1.time; (unsigned) waitTime < solution[handle1].states.size(); waitTime++) {
          if (!solution[handle1].states[waitTime].first.equalExceptTime(stateInizial2)) break;
        }

        startingTime = stateInizial1.time;
        endingTime   = waitTime - 1;
        constrainedState = stateInizial1;
      }
              
      else if (stateInizial2.equalExceptTime(solution[handle2].states[stateInizial2.time-1].first)) {
        int waitTime;

        for (waitTime = stateInizial2.time; (unsigned) waitTime < solution[handle2].states.size(); waitTime++) {
          if (!solution[handle2].states[waitTime].first.equalExceptTime(stateInizial1)) break;
        }

        startingTime = stateInizial1.time;
        endingTime   = waitTime - 1;
        constrainedState = stateInizial2;
      }


      setConstrains(constrainedState, startingTime, endingTime, handle2, accConstraints);
      resultConstraints.insert(std::make_pair(handle2, accConstraints));
      return;
    }

    


    
    // TODO: Proof if everything is fine
    if (checkCriticalLocation(runningStateAgent1)) {

      std::cout << "INFO: Initial State is a critical State!" << std::endl;

      // Kritische Stelle kann eine Goal sein, welches aber nicht zum anderen Agenten gehört

      if (isGoalOfAgent(runningStateAgent1, handle1)) {
        std::cout << "INFO: Initial State is a critical State (Goal)!" << std::endl << std::endl;

        constrainedState = runningStateAgent1;

        // Agent2 would enter the inital State at this time
        startingTime = getPrevPosition(runningStateAgent2, handle2, solution).second + 1;

        endingTime   = runningStateAgent1.time;


        setConstrains(constrainedState, startingTime, endingTime, handle2, accConstraints);
        resultConstraints.insert(std::make_pair(handle2, accConstraints));
        return;
      } 

      const auto& nextActions1 = m_edges[Location(runningStateAgent1.y, runningStateAgent1.x, runningStateAgent1.dir)];
      const auto& nextActions2 = m_edges[Location(runningStateAgent2.y, runningStateAgent2.x, runningStateAgent2.dir)];


      if (nextActions1.size() >= 2) {

        std::cout << "INFO: Initial State is a critical State (Applicable)!" << std::endl << std::endl;

        handleUsableJunction(runningStateAgent1, runningStateAgent2, handle1, nextActions1, solution, resultConstraints);
        
      }

      if (nextActions2.size() >= 2) {

        std::cout << "INFO: Initial State is a critical State (Not Applicable)!" << std::endl << std::endl;

        std::pair<State, int> nextAgent2 = getNextPosition(runningStateAgent2, handle2, solution);
        if (nextAgent2.second == -1) {
          return;
        }

        std::pair<State, int> prevAgent1 = getPrevPosition(runningStateAgent1, handle1, solution);
        if (prevAgent1.second == -1) {
          return;
        }

        handleNotUsableJunction(prevAgent1, nextAgent2, handle2, nextActions2, resultConstraints, solution);
      }
    }


    while (true) {

      Constraints accConstraints;

      std::pair<State, int> nextAgent2 = getNextPosition(runningStateAgent2, handle2, solution);
      if (nextAgent2.second == -1) {
        // We have reached the last State of the solution of the Agent 2
        // There is another previous state of Agent 1, so this Agent needs to wait for the other Agent to 
        // get to the goal and the proceed 

        std::cout << "INFO: There is a goal constraint with A[" << handle1 << "] and B[" << handle2 << "]" << std::endl;

        std::pair<State, int> prevAgent1 = getPrevPosition(runningStateAgent1, handle1, solution);
        // Test if Agent 1 is standing right on the goal and can't escape the situation
        if (prevAgent1.second == -1) {

          // If this is true, there is no way for Agent 1 to come out of this situation and is fully blocking Agent 2 to its goal
          if (m_edges[Location(prevAgent1.first.y, prevAgent1.first.x, prevAgent1.first.dir)].size() < 2) {
            std::cout << "INFO: A[" << handle1 << "] can't get out of this situation" << std::endl;
            break;
          }

          // The Agents is standing on the Goal from Agent 2, but can get out of the way
          constrainedState = getNextPosition(runningStateAgent1, handle1, solution).first;

          // At this time Agent 1 would enter the cell after the goal
          startingTime = constrainedState.time;

          // This is the last State - 1, because the Agent is only one step in the Goal and then vanish
          endingTime   = nextAgent2.first.time - 1;


          // Make sure to constrain the state of the goal, that Agent 1 is not at the position at the time of Agent 2
          setConstrains(nextAgent2.first, nextAgent2.first.time, nextAgent2.first.time, handle1, accConstraints);

        }

        // At this point we have  a normal state before the goal
        else {

          // At this time, Agent 1 would enter the goal of Agent 2
          startingTime = prevAgent1.second + 1;

          // The last timestep of Agent 2
          endingTime   = nextAgent2.first.time;

          // This is the last State (Goal) of the Solution from Agent 2
          constrainedState = nextAgent2.first;

        }
        

        setConstrains(constrainedState, startingTime, endingTime, handle1, accConstraints);

        resultConstraints.insert(std::make_pair(handle1, accConstraints));

        break;
      }

      std::pair<State, int> prevAgent1 = getPrevPosition(runningStateAgent1, handle1, solution);
      if (prevAgent1.second == -1) {
        // We have reached the first (0-th) State of the solution of the Agent 1

        // At this point there is no way of return. The Constraints before must handle this problem

        std::cout << "INFO: Found Not-Possible A[" << handle2 << "]: " << runningStateAgent2 << std::endl;
        break;
      }

      runningStateAgent1 = prevAgent1.first;
      runningStateAgent2 = nextAgent2.first;


      if (!runningStateAgent1.equalExceptTime(runningStateAgent2)) {
        // For the first time the Solution of both Agents do not interfere with each other, so now the Agent 1 needs to 
        // get as much constrains he would enter the next Cell and the other Agent would leave the oder cell

        // Note: With this function there is also the Head Error included, so we do not need to check this further

        std::cout << "INFO: First time the Solution is different!" << std::endl;

        startingTime = prevAgent1.second + 1;
        endingTime   = nextAgent2.second - 1;

        constrainedState = getNextPosition(runningStateAgent1, handle1, solution).first;


        setConstrains(constrainedState, startingTime, endingTime, handle1, accConstraints);
        resultConstraints.insert(std::make_pair(handle1, accConstraints));

        break;
      }

      std::cout << "INFO: Lsg: A[" << handle1 << "]: " << prevAgent1.first << " <-> A[" << handle2 << "]: " << nextAgent2.first << ";" << std::endl;

      
      // Check if the current state is a critical state, where one of both agents can do a action

      if (checkCriticalLocation(runningStateAgent1)) {
  

        const auto& nextActions1 = m_edges[Location(runningStateAgent1.y, runningStateAgent1.x, runningStateAgent1.dir)];
        const auto& nextActions2 = m_edges[Location(runningStateAgent2.y, runningStateAgent2.x, runningStateAgent2.dir)];


        // This junction is not applicable for Agent 2, so constrain Agent 1
        if (nextActions1.size() >= 2) {
          std::cout << "INFO: Found a Critical State A[" << handle1 << "]: " << runningStateAgent1 << std::endl;

          handleUsableJunction(runningStateAgent1, runningStateAgent2, handle1, nextActions1, solution, resultConstraints);

        }



        nextAgent2 = getNextPosition(runningStateAgent2, handle2, solution);
        if (nextAgent2.second == -1) {
          // The critical cell is the goal of the agent, so continue and handle the constrains above
          continue;
        }

        prevAgent1 = getPrevPosition(runningStateAgent1, handle1, solution);
        if (prevAgent1.second == -1) {
          // At this Point there is no way, that Agent 2 can use the critical cell, because agent 1 is standing 
          // right on it, so just continue to get the same result above
          continue;
        }

        // This junction is not applicable for Agent 1, so constrain the other Agent
        if (nextActions2.size() >= 2) {
          std::cout << "INFO: Found a Critical State A[" << handle2 << "]: " << runningStateAgent2 << std::endl;

          // If the have the same location after the critical state and it is not the starting point of Agent 1
          // or the end Point of Agent 2, then we need to constrain Agent 2, that he will take another route

          handleNotUsableJunction(prevAgent1, nextAgent2, handle2, nextActions2, resultConstraints, solution);

          // If the next States are not the same, there is no need to constrain the other agent 
          // Agent 1 will get constrained in the next iteration, where he should stay in the first state
          // the solution is different for the first time
        }
      }

    }
  }

  // runningStateAgent1 / 2: the current critical State of Agent 1/2
  // handle: the handle of Agent 1, because he will get constrained
  // nextActions: the next Actions Agent 1 can take
  // solution: to get the next state of Agent 1
  // resultConstraints: so the Agent 1 will get constrained  
  void handleUsableJunction(const State& runningStateAgent1, const State& runningStateAgent2, const size_t handle, const auto& nextActions, const auto& solution, auto& resultConstraints) {

    State stateAfterCritical = getNextPosition(runningStateAgent1, handle, solution).first;

    int nextStateHeuristic = m_map.getHeuristicValue(handle, stateAfterCritical.y, stateAfterCritical.x, stateAfterCritical.dir);
    int currentHeuristic;

    for (const auto& newLoc: nextActions) {

      if (stateAfterCritical.y == newLoc.y && stateAfterCritical.x == newLoc.x && stateAfterCritical.dir == newLoc.new_dir) continue;

      if (checkSolvableJunction(handle, stateAfterCritical.time, newLoc.y, newLoc.x, solution)) continue;

      currentHeuristic = m_map.getHeuristicValue(handle, newLoc.y, newLoc.x, newLoc.new_dir);

      // At this time Agent 1 would enter the cell after the critical cell
      int startingTime = stateAfterCritical.time;

      // Now the Heuristics difference is added so Agent 1 has to take the other way
      int endingTime   = std::abs(nextStateHeuristic - currentHeuristic) + startingTime + 1;

      // It can be, that the Heuristic is lower than the actual time the other Agent is in the cell after the critical cell
      endingTime   = std::max(runningStateAgent2.time - 1, endingTime);

      State constrainedState = stateAfterCritical;

      Constraints accConstraints;
      setConstrains(constrainedState, startingTime, endingTime, handle, accConstraints);

      resultConstraints.insert(std::make_pair(handle, accConstraints));
    }
  }

  // prevAgent1: the State which comes before runningStateAgent1
  // nextAgent2: the State which comes after  runningStateAgent2
  // handle: the handle of Agent 2 because he will get constrained
  // nextActions: all the Actions Agent 2 can do
  // resultConstraints: so the Agent 2 will get constrained
  void handleNotUsableJunction(const auto& prevAgent1, const auto& nextAgent2, const size_t handle, const auto& nextActions, auto& resultConstraints, const auto& solution) {
    if (prevAgent1.first.equalExceptTime(nextAgent2.first)) {
      std::cout << "INFO: Same way A[" << handle << "]: " << nextAgent2.first << std::endl;

      int nextStateHeuristic = m_map.getHeuristicValue(handle, nextAgent2.first.y, nextAgent2.first.x, nextAgent2.first.dir);
      int currentHeuristic;

      for (const auto& newLoc: nextActions) {

        if (nextAgent2.first.y == newLoc.y && nextAgent2.first.x == newLoc.x && nextAgent2.first.dir == newLoc.new_dir) continue;

        if (checkSolvableJunction(handle, nextAgent2.first.time, newLoc.y, newLoc.x, solution)) continue;

        currentHeuristic = m_map.getHeuristicValue(handle, newLoc.y, newLoc.x, newLoc.new_dir);

        int startingTime = nextAgent2.second;
        int endingTime   = std::abs(nextStateHeuristic - currentHeuristic) + startingTime + 1;

        State constrainedState = nextAgent2.first;


        Constraints accConstraints;
        setConstrains(constrainedState, startingTime, endingTime, handle, accConstraints);
        resultConstraints.insert(std::make_pair(handle, accConstraints)); 
      }
    }
  }



  bool checkSolvableJunction(const size_t givenHandle, const int startingTime, const int y, const int x, const auto& solution) {

    for (size_t handle = 0; handle < solution.size(); handle++) {

      if (handle == givenHandle) continue;

      std::cout << "NEW INFO: Searching A[" << handle << "], by starting with " << startingTime << std::endl;

      for (int t = startingTime; (unsigned)t < solution[handle].states.size(); t++) {
        
        State currentState = getState(handle, solution, t);

        std::cout << currentState << std::endl;

        if (currentState.x == x && currentState.y == y) {
          std::cout << "NEW INFO: Found the Same State!" << std::endl;
          return false;
        }

      }
    }


    return true;
  }


  bool getFirstConflict(const std::vector<PlanResult<State, Action, int> >& solution, std::multimap<size_t, Constraints>& resultConstraints) {
    
    // Calculate the max time
    int max_t = 0;
    for (const auto& sol : solution) {
      max_t = std::max<int>(max_t, sol.states.size() - 1);
    }


    for (int t = 0; t < max_t; t++) {
      // check drive-drive vertex collisions
      for (size_t handle1 = 0; handle1 < solution.size(); handle1++) {
        if ((unsigned)t >= solution[handle1].states.size()) continue;

        State stateInizial1 = getState(handle1, solution, t);

        for (size_t handle2 = handle1 + 1; handle2 < solution.size(); handle2++) {
          if ((unsigned)t >= solution[handle2].states.size()) continue;

          State stateInizial2 = getState(handle2, solution, t);

          // At this point there was an Vertex error, handle it in the corresponding function 
          if (stateInizial1.equalExceptTime(stateInizial2)) {
            
            handleConflicts(stateInizial1, stateInizial2, handle1, handle2, solution, resultConstraints, 0);
            handleConflicts(stateInizial2, stateInizial1, handle2, handle1, solution, resultConstraints, 0);

            return true;
          }
        }
      }

      // drive-drive edge (swap)
      for (size_t handle1 = 0; handle1 < solution.size(); handle1++) {
        if ((unsigned)t >= solution[handle1].states.size()-1) continue;  // Make sure to use -1, otherwise it will take the false solution

        State stateInizial1a = getState(handle1, solution, t);

        std::pair<State, int> resultStateInizial1b = getNextPosition(stateInizial1a, handle1, solution);
        if (resultStateInizial1b.second == -1) {
          std::cout << "ACHTUNG: Es wurde kein nachvolgender State in getFirstConflict()!" << std::endl;
          continue;
        }

        State stateInizial1b = resultStateInizial1b.first;

        for (size_t handle2 = handle1 + 1; handle2 < solution.size(); handle2++) {
          if ((unsigned)t >= solution[handle2].states.size()-1) continue;

          State stateInizial2a = getState(handle2, solution, t);
          

          std::pair<State, int> resultStateInizial2b = getNextPosition(stateInizial2a, handle2, solution);
          if (resultStateInizial2b.second == -1) {
            std::cout << "ACHTUNG: Es wurde kein nachvolgender State in getFirstConflict()!" << std::endl;
            continue;
          }

          State stateInizial2b = resultStateInizial2b.first;


          // If there is a Edge Conflict
          if (stateInizial1a.equalExceptTime(stateInizial2b) && stateInizial1b.equalExceptTime(stateInizial2a)) {
            handleConflicts(stateInizial1b, stateInizial2b, handle1, handle2, solution, resultConstraints, 1);
            handleConflicts(stateInizial2b, stateInizial1b, handle2, handle1, solution, resultConstraints, 1);

            return true;
          }
        }
      }
    }

    return false;
  }

  void onExpandHighLevelNode(int /*cost*/) { m_highLevelExpanded++; }

  void onExpandLowLevelNode(const State& /*s*/, int /*fScore*/,
                            int /*gScore*/) {
    m_lowLevelExpanded++;
  }

  int highLevelExpanded() { return m_highLevelExpanded; }

  int lowLevelExpanded() const { return m_lowLevelExpanded; }

 private:
  State getState(size_t agentIdx,
                 const std::vector<PlanResult<State, Action, int> >& solution,
                 size_t t) {
    // assert(agentIdx < solution.size());
    // if (t < solution[agentIdx].states.size()) {
    
    return solution[agentIdx].states[t].first;
    // }
    // assert(!solution[agentIdx].states.empty());
    // return solution[agentIdx].states.back().first;
  }

  bool stateValid(const State& s) {
    assert(m_constraints);
    const auto& con = m_constraints->vertexConstraints;

    // It is true, when the current state has not the same time and same position
    bool test_constraint = con.find(VertexConstraint(s.time, s.y, s.x)) == con.end();

    // Also check if the States are in the dimension of the given grid and return the truth value
    return s.x >= 0 && s.x < m_dimx && s.y >= 0 && s.y < m_dimy && test_constraint;
  }

  bool transitionValid(const State& s1, const State& s2) {
    assert(m_constraints);
    const auto& con = m_constraints->edgeConstraints;
    return con.find(EdgeConstraint(s1.time, s1.y, s1.x, s2.y, s2.x)) == con.end();
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
  desc.add_options()("help", "produce help message")
    ("input,i", po::value<std::string>(&inputFile)->required(), "input file (YAML)")
    ("inputGrid,g", po::value<std::string>(&inputGridFile)->required(), "inputGrid file (txt)")
    ("output,o", po::value<std::string>(&outputFile)->required(), "output file (YAML)");

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


  // Loading and initializing the Map with its grind and the grid actions (TODO)
  Map map(inputFile, inputGridFile);
  map.inizialize();


  YAML::Node config = YAML::LoadFile(inputFile);

  std::unordered_map<Location, std::vector<NewLocation>> edges;
  std::vector<Location> critical;

  std::vector<Location> goals;
  std::vector<State> startStates;

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
    const auto& start = node["start"];

    const auto& direction = node["direction"];

    const auto& goal = node["goal"];
    startStates.emplace_back(State(0, start[0].as<int>(), start[1].as<int>(), direction.as<int>()));

    goals.emplace_back(Location(goal[0].as<int>(), goal[1].as<int>()));
  }

  // Create the Environment based on the collected parameters and search with CBS
  Environment mapf(dimy, dimx, edges, goals, map, critical);
  CBS<State, Action, int, Conflict, Constraints, Environment> cbs(mapf);
  std::vector<PlanResult<State, Action, int> > solution;

  Timer timer;
  bool success = cbs.search(startStates, solution);
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
    return 1;
  }

  return 0;
}
