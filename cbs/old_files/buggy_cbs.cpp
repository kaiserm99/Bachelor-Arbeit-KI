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

  void getNeighbors(const State& s,
                    std::vector<Neighbor<State, Action, int> >& neighbors) {

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

  State getLastState(const int handle, const std::vector<PlanResult<State, Action, int>>& solution) {
    return solution[handle].states[solution[handle].states.size() - 1].first;
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

      std::cout << "Crit State: " << checkState << " from Agent " << handle1 << std::endl;

      // If the critical Point is the goal of the other agent, then return this one
      if (checkState.x == m_goals[handle2].x && checkState.y == m_goals[handle2].y && t_back != 0) {
        if (m_debug) std::cout << "TEST: Agent " << handle1 << " lässt Agent "<< handle2 << " zur Lösung durch" << std::endl;
        return checkState;
      }

      // If it is a critical state, then check if minimum one of the agents can use it
      if (m_edges[Location(checkState.y, checkState.x, checkState.dir)].size() >= 2) {
        return checkState;  
      }

      // Check if the other agent can use this cell. If this is -1, it can be, that the second agent never gets to the critical cell
      int tOther = getFirstTimeToState(t, checkState, handle2, solution);
      if (tOther == -1) {
        std::cout << "This should never trigger" << std::endl;
        continue;
      }


      if (m_edges[Location(checkState.y, checkState.x, solution[handle2].states[tOther].first.dir)].size() >= 2) {
        return checkState;
      }

      // If there was no return statement and both of the agents can't use the critical point, go to the other critical point

    }

    return State(-1, -1, -1, -1);
  }


  void addConstraints(const int t, size_t handle1, const size_t handle2, 
                      const State& critState, const std::vector<PlanResult<State, Action, int>>& solution,
                      Constraints& cons, Constraints& consOther,
                      int type, int edge = 0) {


    // t: Zeitpunkt, an dem der jeweilige Fehler aufgetreten ist
    // handle1, handle2: die Nummern der Agenten
    // critState: der kritische State (kann ein Goal und/oder eine normale Abzweigung sein)
    // cons: die Constrains des Agenten mit handle1
    // consOhter: die Constrains des Agenten mit handle2
    // type: gibt an, ob die gefundene kritische Stelle für den Agenten1 anwendbar ist oder nicht (1 --> ja, 0 --> nein)
    // edge: gibt den jeweiligen Fehlertyp an (1 --> edge error, 0 --> vertex)


    // An dieser Stelle wurde schon überprüft, ob ein Head Error vorliegt. Dies ist hier nicht mehr der Fall und im folgenden
    // wird angenommen, dass die beiden Agenten aufeinander zufahren und Agent2 auch die kritische Stelle passieren muss


    std::cout << "INFO: Constraints für Agent " << handle1 << " und Agent " << handle2 << " werden gesetzt:" << std::endl << std::endl;
    

    // Also proof the initial position of the agent. If it is a critical point and also applicable for the agent, then set the constraints
    // to the t=1 state, so the agent must take the other way
    if (critState.time == 0 && type == 0) {
      std::cout << "There is no way to come out of this situation!" << std::endl;
      return;
    }

    int startingTime;
    int endingTime;

    // Initialize
    State constrainedState = critState;

    // Make sure to calculate the corresponding critical State from Agent2, because critState has a false timestep
    int tOther = getFirstTimeToState(t, critState, handle2, solution);
    if (tOther == -1) {
      std::cout << "This should never trigger" << std::endl;
    }

    // This is the first appearance of the critical state in the solution of Agent 2
    State critOtherState = solution[handle2].states[tOther].first;


    // Die Zeit des Konflikts ist auch die Zeit der kritischen Stelle


    // ACHTUNG: FEHLERHAFT
    if (t == critState.time) {


      if (type == 1) {

        std::cout << "INFO: Zero handling Error (type == 1)!" << std::endl;

        /*
        std::pair<State, int> resultAgent2Starting = getPrevPosition(critOtherState, handle2, solution);
        if (resultAgent2Starting.second == -1) {
          std::cerr << "ACHTUNG: Es wurde kein vorheriger State im Zero handling (type == 1) gefunden!" << std::endl;
          return;
        }
        */

        std::pair<State, int> resultAgent1Ending = getNextPosition(critState, handle1, solution);
        if (resultAgent1Ending.second == -1) {
          std::cerr << "ACHTUNG: Es wurde kein nachvolgender State im Zero handling (type == 1) gefunden!" << std::endl;
          return;
        }

        // startingTime = resultAgent2Starting.second + 1;
        startingTime = critOtherState.time;

        endingTime   = resultAgent1Ending.second - 1 + edge;

        // Der andere Agent wird bestraft mit den berechneten Parameter
        cons = consOther;

        // Only for printing the other handle when this case happened
        handle1 = handle2;
      }

      else if (type == 0) {

        std::cout << "INFO: Zero handling  Error (type == 0)!" << std::endl;

        std::pair<State, int> resultAgent1Starting = getPrevPosition(critState, handle1, solution);
        if (resultAgent1Starting.second == -1) {
          std::cout << "ACHTUNG: Es wurde kein vorheriger State im Zero handling (type == 0) gefunden!" << std::endl;
          return;
        }


        std::pair<State, int> resultAgent2Ending = getNextPosition(critOtherState, handle2, solution);
        if (resultAgent2Ending.second == -1) {
          std::cerr << "ACHTUNG: Es wurde kein nachvolgender State im Zero handling (type == 0) gefunden!" << std::endl;
          return;
        }

        startingTime = resultAgent1Starting.second + 1;
        endingTime   = resultAgent2Ending.second   - 1;
      }

    }


    // Wenn die kritische Stelle das Ziel des anderen Agenten ist
    else if (isGoalOfAgent(critState, handle2)) {

      // An dieser Stelle muss Agent 1 den Agenten 2 durchlassen

      std::cout << "INFO: There is a Goal constraint!" << std::endl;

      // ... und ist anwendbar für den Agent1
      if (type == 1) {

        std::pair<State, int> resultAgent1 = getNextPosition(critState, handle1, solution);
        if (resultAgent1.second == -1) {
          std::cerr << "ACHTUNG: Es wurde kein nachvolgender State im Goal handeling gefunden!" << std::endl;
          return;
        }

        std::pair<State, int> resultAgent2 = getPrevPosition(critOtherState, handle2, solution);
        if (resultAgent2.second == -1) {
          std::cerr << "ACHTUNG: Es wurde kein vorheriger State im Goal handeling gefunden!" << std::endl;
          return;
        }


        // Zeitpunkt, an dem Agent 1 der Position nach dem Ziel betreten würde
        startingTime = resultAgent1.second;
        endingTime   = resultAgent2.second;

        // Die nachvolgende Position nach dem Ziel des Agenten 2
        constrainedState = resultAgent1.first;

        // Constraine auch noch das Eigentliche Ziel
        State lastStateAgent2 = getLastState(handle2, solution);
        cons.vertexConstraints.emplace(VertexConstraint(lastStateAgent2.time, lastStateAgent2.y, lastStateAgent2.x));

      }


      // ... und ist nicht Anwendbar für Agent1
      else if (type == 0) {

        std::pair<State, int> resultAgent1 = getPrevPosition(critState, handle1, solution);
        if (resultAgent1.second == -1) {
          std::cout << "ACHTUNG: Agent " << handle1 << " steht auf dem Ziel und kann dem nicht entkommen!"  << std::endl;
          return;
        }


        startingTime = resultAgent1.second + 1;
        endingTime   = getLastState(handle2, solution).time;

      }
    
    }


    // TODO: isGoalOfAgent(critState, handle2) && edge == 1


    
    
    else if (type == 1) {

      std::cout << "INFO: Normal handeling Error (type == 1)!" << std::endl;

      std::pair<State, int> resultAgent1 = getNextPosition(critState, handle1, solution);
      if (resultAgent1.second == -1) {
        std::cerr << "ACHTUNG: Es wurde kein nachvolgender State im normalen handeling (edge == 0, type == 1) gefunden!" << std::endl;
        return;
      }

      std::pair<State, int> resultAgent2 = getPrevPosition(critOtherState, handle2, solution);
      if (resultAgent2.second == -1) {
        std::cerr << "ACHTUNG: Es wurde kein vorheriger State im normalen handeling (edge == 0, type == 1) gefunden!" << std::endl;
        return;
      }

      // This is the state after the critical cell
      constrainedState = resultAgent1.first;

      // This is the time Agent1 would enter the cell after the critical cell
      startingTime     = resultAgent1.second;

      // This it the time Agent2 would leave the cell after the critical cell
      endingTime       = resultAgent2.second;


    }
    
    else if (type == 0) {

      std::cout << "INFO: Normal handeling Error (type == 0)!" << std::endl;

      std::pair<State, int> resultAgent1 = getPrevPosition(critState, handle1, solution);
      if (resultAgent1.second == -1) {
        std::cerr << "ACHTUNG: Es wurde kein vorheriger State im normalen handeling (edge == 0, type == 0) gefunden!" << std::endl;
        return;
      }

      std::pair<State, int> resultAgent2 = getNextPosition(critOtherState, handle2, solution);
      if (resultAgent2.second == -1) {
        std::cerr << "ACHTUNG: Es wurde kein nachvolgender State im normalen handeling (edge == 0, type == 0) gefunden!" << std::endl;
        return;
      }

      // Make sure to constrain the critical cell (initial) 

      // This is the time Agent1 would enter the critical cell
      startingTime = resultAgent1.second + 1;

      // This is the time Agent2 would leave the critical cell
      endingTime   = resultAgent2.second - 1;

    }


    // Finally set all the constrains which got calculated previously
    setConstrains(constrainedState, startingTime, endingTime, handle1, cons);

  }


  void setConstrains (const State& constrainedState, const int startingTime, const int endingTime, const size_t handle, Constraints& cons) {

    if (true) std::cout << "Set VertexConstraint from " << startingTime << " to " << endingTime << std::endl;


    for (int timeVertex = startingTime; timeVertex <= endingTime; timeVertex++) {

      cons.vertexConstraints.emplace(VertexConstraint(timeVertex, constrainedState.y, constrainedState.x));
      if (m_debug) std::cout << "Agent " << handle << ": " << VertexConstraint(timeVertex, constrainedState.y, constrainedState.x) << std::endl;

    }
  }


  // This function takes a state and a handle and calculates the next possible state in the solution, because the
  // state in the next timestep can also be the same position

  // This functions returns a Pair with the next State and the timestep of this 
  std::pair<State, int> getNextPosition(const State& state, const int handle, const std::vector<PlanResult<State, Action, int>>& solution) {

    int currTime;

    // Start at the State in the next timestep and loop trough it, until there was another position
    for (currTime = state.time+1; (unsigned)currTime < solution[handle].states.size(); currTime++) {

      State currState = solution[handle].states[currTime].first;

      if (state.y != currState.y || state.x != currState.x) return std::make_pair(currState, currTime);
    }


    std::cerr << "ACHTUNG: In getNextPosition() gab es keinen Nachvolgerstate!!" << std::endl;

    // Make sure to handle this exception in the code, because it was already the last position of the the solution
    return std::make_pair(solution[handle].states[currTime].first, -1);
  }


  std::pair<State, int> getPrevPosition(const State& state, const int handle, const std::vector<PlanResult<State, Action, int>>& solution) {

    for (int currTime = state.time-1; currTime >= 0; currTime--) {

      State currState = solution[handle].states[currTime].first;

      // std::cout << currState << std::endl;

      if (state.y != currState.y || state.x != currState.x) return std::make_pair(currState, currTime);
    }

    std::cerr << "ACHTUNG: In getPrevPosition() gab es keine vorherige Position!!" << std::endl;

    return std::make_pair(solution[handle].states[0].first, -1);
  }


  // This function returns the timestep of the first appearence of the given state in the solution of the given handle
  int getFirstTimeToState(const int time, const State& critState, const size_t handle, const std::vector<PlanResult<State, Action, int>>& solution) {

    for (int currTime = time; (unsigned)currTime < solution[handle].states.size(); currTime++) {

      State currState = solution[handle].states[currTime].first;

      // Wenn die Lösung die Position der kritischen Stelle von Agent 1 erreicht hat, breche ab
      if (currState.equalExceptTime(critState)) {

        std::cout << "Agent " << handle << " kritische Stelle: " << currState << std::endl;

        return currTime;
      }
    }

    std::cout << "There was an error in getFirstTimeToState() and the Agent " << handle << " never reached the critical State!!" << std::endl;

    return -1;
  }


  bool checkHeadError(const State& errorState, const size_t handle1, const size_t handle2, const std::vector<PlanResult<State, Action, int>>& solution) {

    std::pair<State, int> resultAgent1 = getPrevPosition(errorState, handle1, solution);
    if (resultAgent1.second == -1) {
      std::cerr << "ACHTUNG: Es wurde kein vorheriger State in checkHeadError() gefunden!" << std::endl;
      return false;
    }

    std::pair<State, int> resultAgent2 = getNextPosition(errorState, handle2, solution);
    if (resultAgent2.second == -1) {
      std::cerr << "ACHTUNG: Es wurde kein nachvolgender State in checkHeadError() gefunden!" << std::endl;
      return false;
    }

    if (resultAgent1.first.y != resultAgent2.first.y || resultAgent1.first.x != resultAgent2.first.x) {
      
      return true;
    }

    return false;
  }


  void handleConflicts(const State& stateInizial1, const State& stateInizial2, int t, const size_t handle1, const size_t handle2, const std::vector<PlanResult<State, Action, int>>& solution, std::map<size_t, Constraints>& resultConstraints, int edge) {

    // Only for Debug purposes
    if (edge == 0) {
      if (m_debug) std::cout << std::endl << "Vertex Error at: " << stateInizial1 << " and " << stateInizial2 << std::endl;
    } else {
      if (m_debug) std::cout << std::endl << "Edge Error at: " << stateInizial1 << " and " << stateInizial2 << std::endl;
    }

    // Get to the critical states
    State state1 = getCriticalState(t, handle1, handle2, solution);

    State state2 = getCriticalState(t, handle2, handle1, solution);

    if (m_debug) std::cout << "Agent: " << handle1 << " Conflict: " << state1 << std::endl;
    if (m_debug) std::cout << "Agent: " << handle2 << " Conflict: " << state2 << std::endl;



    Constraints accConstraints1;
    Constraints accConstraints2;


    // If there was an Vertex Error and the direction of the error field was the same, then one of the agents waited and the other one drove on the waiting agent
    if (stateInizial1.dir == stateInizial2.dir && edge == 0) {
      
      // Agent1 is waiting for another so, the previous agent has to wait
      if (stateInizial1.equalExceptTime(solution[handle1].states[stateInizial1.time-1].first)) {
        if (true) std::cout << "Agent " << handle1 << " " << stateInizial1 << " und " << solution[handle1].states[stateInizial1.time-1].first << std::endl;

        // Loop troth the vertex error time and check how long the agent has to wait and add the constraints to the other agents
        for (int waitTime = t; (unsigned)waitTime < solution[handle1].states.size(); waitTime++) {

          if (!solution[handle1].states[waitTime].first.equalExceptTime(stateInizial2)) break;

          accConstraints2.vertexConstraints.emplace(VertexConstraint(waitTime, stateInizial1.y, stateInizial1.x));
        }
      }

        
              
      else if (stateInizial2.equalExceptTime(solution[handle2].states[stateInizial2.time-1].first)) {
        if (true) std::cout << "Agent " << handle2 << " " << stateInizial2 << " und " << solution[handle2].states[stateInizial2.time-1].first << std::endl;


        for (int waitTime = t; (unsigned)waitTime < solution[handle2].states.size(); waitTime++) {

          if (!solution[handle2].states[waitTime].first.equalExceptTime(stateInizial1)) break;

          accConstraints1.vertexConstraints.emplace(VertexConstraint(waitTime, stateInizial2.y, stateInizial2.x));

        }
      }
    }

    // There is no way the agent can get over each other, so return true and don't expand this HighLevelNode further
    else if (state1.time == -1 && state2.time == -1) {
      std::cout << "Agent: "<<  handle1 << " & Agent: " <<  handle2 << " can't pass each other!" << std::endl;
    }

    else if (state1.time == -1) {  // Agent 1 cant get out of the situation 

      if (m_edges[Location(state2.y, state2.x, state2.dir)].size() >= 2) {

        if (m_debug) std::cout << "TEST 1a" << std::endl;
        addConstraints(t, handle2, handle1, state2, solution, accConstraints2, accConstraints1, 1, edge);


      } else {

        if (m_debug) std::cout << "TEST 1b" << std::endl;
        addConstraints(t, handle2, handle1, state2, solution, accConstraints2, accConstraints2, 0, edge);

      }
    }

    else if (state2.time == -1) {  // Agent 2 cant get out of the situation 


      if (m_edges[Location(state1.y, state1.x, state1.dir)].size() >= 2) {

        if (m_debug) std::cout << "TEST 2a" << std::endl;
        addConstraints(t, handle1, handle2, state1, solution, accConstraints1, accConstraints2, 1, edge);

      } else {

        if (m_debug) std::cout << "TEST 2b" << std::endl;
        addConstraints(t, handle1, handle2, state1, solution, accConstraints1, accConstraints1, 0, edge);
      }
    }

    else {  // Both Agents can get out of the situation


      if (m_edges[Location(state1.y, state1.x, state1.dir)].size() >= 2) {

        if (m_debug) std::cout << "TEST 3a" << std::endl;
        addConstraints(t, handle1, handle2, state1, solution, accConstraints1, accConstraints2, 1, edge);

      } else {

        if (m_debug) std::cout << "TEST 3b" << std::endl;
        addConstraints(t, handle1, handle2, state1, solution, accConstraints1, accConstraints1, 0, edge);

      }


      if (m_edges[Location(state2.y, state2.x, state2.dir)].size() >= 2) {

        if (m_debug) std::cout << "TEST 4a" << std::endl;
        addConstraints(t, handle2, handle1, state2, solution, accConstraints2, accConstraints1, 1, edge);

      } else {

        if (m_debug) std::cout << "TEST 4b" << std::endl;
        addConstraints(t, handle2, handle1, state2, solution, accConstraints2, accConstraints2, 0, edge);

      }
    }

    resultConstraints[handle1] = accConstraints1;
    resultConstraints[handle2] = accConstraints2;
  }



  bool getFirstConflict(const std::vector<PlanResult<State, Action, int> >& solution, std::map<size_t, Constraints>& resultConstraints) {
    
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

            if (checkHeadError(stateInizial1, handle1, handle2, solution)) {

              Constraints accConstraints;

              std::cout << "INFO: Head Error!" << std::endl;

              setConstrains(stateInizial1, getPrevPosition(stateInizial1, handle2, solution).second + 1, getNextPosition(stateInizial1, handle2, solution).second - 1, handle1, accConstraints);
              resultConstraints[handle1] = accConstraints;
            }

            else {

             handleConflicts(stateInizial1, stateInizial2, t, handle1, handle2, solution, resultConstraints, 0);

            }

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
            handleConflicts(stateInizial1a, stateInizial2a, t, handle1, handle2, solution, resultConstraints, 1);

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
















/*
      // ... und es liegt ein Edge Error vor
      else if (edge == 1) {

        

        if (type == 1) {

          std::cout << "INFO: Zero handeling Edge Error (type == 1)!" << std::endl;
          

          std::pair<State, int> resultAgent1Ending = getNextPosition(critState, handle1, solution);
          if (resultAgent1Ending.second == -1) {
            std::cerr << "ACHTUNG: Es wurde kein nachvolgender State im Zero handeling (edge == 1, type == 1) gefunden!" << std::endl;
            return;
          }

          startingTime = t;
          endingTime   = resultAgent1Ending.second - 1;

          // Der andere Agent wird bestraft mit den berechneten Parameter
          cons = consOther;

          // Only for printing the other handle when this case happened
          handle1 = handle2;

        } else if (type == 0) {

          std::cout << "INFO: Zero handeling Edge Error (type == 0)!" << std::endl;

          std::pair<State, int> resultAgent2Ending = getNextPosition(critOtherState, handle2, solution);
          if (resultAgent2Ending.second == -1) {
            std::cerr << "ACHTUNG: Es wurde kein nachvolgender State im Zero handeling (edge == 1, type == 0) gefunden!" << std::endl;
            return;
          }

          startingTime = t;
          endingTime   = resultAgent2Ending.second - 1;
        }

      }
      */