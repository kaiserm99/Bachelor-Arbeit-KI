/* Copyright 2021, University of Freiburg
 * Bachelorarbeit - Foundations of Artificial Intelligence
 *
 * Author: Marco Kaiser <kaiserm@informatik.uni-freiburg.de>
 *
 * Description: 
 *
 * Usage: 
 */

#include <fstream>
#include <iostream>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <boost/functional/hash.hpp>

#include <boost/python/numpy.hpp>
#include <boost/python.hpp>

namespace p = boost::python;
namespace np = boost::python::numpy;

// Warning: This sequence of includes is important
#include <main_includes.hpp>
#include <constraint.hpp>
#include <cbs.hpp>
#include <agent.hpp>
#include <flatland.hpp>




typedef Neighbor <Action, State> Neighbor_t;
typedef Agent <State, GridLocation> Agent_t;
typedef SearchCBS <FlatlandCBS, State, GridLocation, Agent_t, Action> SearchCBS_t;


FlatlandCBS::FlatlandCBS(p::object railEnv) : m_railEnv(railEnv), m_map(p::extract<np::ndarray>(railEnv.attr("distance_map").attr("get")())) {

	// Get the Rail system of the current Map
	m_rail = p::extract<p::object>(railEnv.attr("rail"));

	// Get the Dimensions of the current Map
	m_dimy = p::extract<int>(railEnv.attr("height"));
	m_dimx = p::extract<int>(railEnv.attr("width"));

	std::cout << "Loading Map with (" << m_dimy << " x " << m_dimx << ")" << std::endl;


	// Get the List with all the agents
	p::list as = p::extract<p::list>(railEnv.attr("agents"));

	for (int handle = 0; handle < len(as); handle++) {

	  // Set the Staring State for every Agent
	  p::tuple start = p::extract<p::tuple>(as[handle].attr("initial_position"));
	  int dir = p::extract<int>(as[handle].attr("initial_direction"));

	  State initialState = State(0, p::extract<int>(start[0]), p::extract<int>(start[1]), dir);

	  // Set the targets for every Agent
	  p::tuple target = p::extract<p::tuple>(as[handle].attr("target"));
	  GridLocation targetLocation = GridLocation(p::extract<int>(target[0]), p::extract<int>(target[1]));

    // Get the speed of the Agent and how many steps he has left in the current initial state (TODO: Maybe)
    p::dict speedDict = p::extract<p::dict>(as[handle].attr("speed_data"));

    int speed =  1 / p::extract<float>(speedDict.get("speed"));

    // Make sure to insert the newly created Agent into the handeled list
    Agent<State, GridLocation> agent(handle, initialState, targetLocation, speed, m_map[handle]);
    agents.emplace_back(agent);
	}


	// Get all the possible actions in the Environment, based on the state and the direction
	std::vector<int> possibleTransitions;
	for (int y = 0; y < m_dimy; y++) {
		for (int x = 0; x < m_dimx; x++) {
	 		for (int dir = 0; dir < 4; dir++) {
		  	possibleTransitions = getTransitions(GridLocation(y, x), dir);
				
				if (all(possibleTransitions)) { continue; }

				possibleActions[GridLocation(y, x, dir)] = getNextGridLocations(GridLocation(y, x), dir, possibleTransitions);
	 		}
		}
	}

  // For Statistics
  nodeExpandCount = 0;
}

// Just sets the Constraints to the given, so the AStar can get the right nieghbours
void FlatlandCBS::setConstraints(const Constraints* constraints) {
  m_constraints = constraints;
}


void FlatlandCBS::getNeighbors(const State& s, const int speed, std::vector<Neighbor <Action, State> >& neighbors) {

	neighbors.clear();

	// Always append the waiting state
	State n(s.time + speed, s.y, s.x, s.dir);

  // Only check the waits with one timestep
  if (m_constraints->checkState(n, 1)) neighbors.emplace_back(Neighbor_t(n, Action::Wait, 1));
 
  const auto& next_actions = possibleActions[GridLocation(s.y, s.x, s.dir)];

  for (const auto& next_action : next_actions) {

    // Create the new State, which is checked in the following
    State n(s.time + speed, next_action.y, next_action.x, next_action.dir);

    // If the State is not valid --> continue and do not insert
    if (!m_constraints->checkState(n, speed)) continue;

    switch (next_action.action) {
      case 1: neighbors.emplace_back(Neighbor_t(n, Action::Left, 1)); break;

      case 2: neighbors.emplace_back(Neighbor_t(n, Action::Up, 1)); break;

      case 3: neighbors.emplace_back(Neighbor_t(n, Action::Right, 1)); break;
    }
  }
}


// Gets a vector of the next possible Locations with the newly updated directions
std::vector<NewGridLocation> FlatlandCBS::getNextGridLocations(const GridLocation& loc, const int direction, std::vector<int>& possibleTransitions) {
		
	std::vector<NewGridLocation> result;

  // (direction-1) % 4 == -1 and not 3 as intended
	std::vector<int> directions{(4 + (direction-1)) % 4, direction, (direction+1) % 4, (direction+2) % 4};
	std::vector<int> actions{1, 2, 3, 2};

	for (int dir = 0; dir < 4; dir++) {

		if (possibleTransitions[directions[dir]]) {

			const GridLocation acc = getGridLocation(loc.y, loc.x, directions[dir]);
			result.emplace_back(NewGridLocation(acc.y, acc.x, directions[dir], actions[dir]));
		}
	}

	return result;
}

// Just calculates the next location on the grid, based on the current location and the direction
GridLocation FlatlandCBS::getGridLocation(const int y, const int x, const int direction) {
	switch (direction) {
		case 0: return GridLocation(y-1, x  ); break;
	  case 1: return GridLocation(y  , x+1); break;
	  case 2: return GridLocation(y+1, x  ); break;
		case 3: return GridLocation(y  , x-1); break;
	}
	assert(0);
}

// Get a vector of Transitions, so for a certain location and direction you can choose where to go next
std::vector<int> FlatlandCBS::getTransitions(const GridLocation& loc, const int direction) {

	std::vector<int> result(4);

	p::tuple possible_transitions = p::extract<p::tuple>(m_rail.attr("get_transitions")(loc.y, loc.x, direction));

	for (int dir = 0; dir < 4; dir++) {
	  const char* currentValue = p::extract<char const *>(p::str(possible_transitions[dir]));
		result[dir] = boost::lexical_cast<int>(currentValue);
	}

	return result;
}

// True, iff the given vector only consits of zeros
bool FlatlandCBS::all(const std::vector<int>& v) {
	return std::all_of(v.begin(), v.end(), [](int i) { return i==0; });
}

// Only for Statistics
void FlatlandCBS::onExpandNode() { nodeExpandCount++; }
void FlatlandCBS::onExpandHighLevelNode() { highLevelNodeExpandCount++; }


State FlatlandCBS::getCriticalState(int conflictTime, size_t handle1, size_t handle2, const std::vector<PlanResult <Action, State> >& solution) {

  for (size_t t_back = conflictTime; t_back >= 0; t_back--) {

    State checkState = solution[handle1].states[t_back];

    if (m_edges[Location(checkState.y, checkState.x, checkState.dir)].size() < 2) continue;

  }

  return defaultState;
}


void FlatlandCBS::handleConflicts(const State& conflictState1, const State& conflictState2, const int conflictTime, const size_t handle1, const size_t handle2, std::vector<PlanResult <Action, State> >& solution, std::vector<std::pair<size_t, Constraints>>& resultConstraints, std::vector<Constraints>& constraints, bool edge) {

  if (!edge) std::cout << std::endl << "[t=" << conflictTime << "] Vertex conflict at: " << conflictState1 << ", " << conflictState2 << std::endl;
  else std::cout << std::endl << "[t=" << conflictTime << "] Edge conflict at: " << conflictState1 << ", " << conflictState2 << std::endl;
}


bool FlatlandCBS::getFirstConflict(std::vector<PlanResult <Action, State> >& solution, std::vector<std::pair<size_t, Constraints>>& resultConstraints, std::vector<Constraints>& constraints) {

  size_t max_t = 0;
  for (const auto& sol : solution) {
    max_t = std::max<size_t>(max_t, sol.states.size() - 1);
  }


  // We can start from time 1, because no agent is overlapping by definition
  for (size_t time = 1; time < max_t; time++) {
    
    // Check for vertex conflicts
    for (size_t handle1 = 0; handle1 < solution.size(); handle1++) {

      if (time >= solution[handle1].states.size()) continue;

      for (size_t handle2 = handle1 + 1; handle2 < solution.size(); handle2++) {
        
        if (time >= solution[handle2].states.size()) continue;

        // Get the possible conflict states
        State conflictState1 = solution[handle1].states[time];
        State conflictState2 = solution[handle2].states[time];


        // Found a Vertex error
        if (conflictState1.equalExceptTime(conflictState2)) {

          // TODO: Check for an head error
          handleConflicts(conflictState1, conflictState2, time, handle1, handle2, solution, resultConstraints, constraints, 0);

          return true;
        }

      }
    }


    // Check for edge conflicts
    for (size_t handle1 = 0; handle1 < solution.size(); handle1++) {

      if (time >= solution[handle1].states.size()) continue;

      for (size_t handle2 = handle1 + 1; handle2 < solution.size(); handle2++) {
        
        if (time >= solution[handle2].states.size()) continue;

        State conflictState1a = solution[handle1].states[time-1];
        State conflictState1b = solution[handle1].states[time];

        State conflictState2a = solution[handle2].states[time-1];
        State conflictState2b = solution[handle2].states[time];

        // Found a Edge error
        if (conflictState1a.equalExceptTime(conflictState2b) && conflictState1b.equalExceptTime(conflictState2a)) {

          handleConflicts(conflictState1b, conflictState2b, time, handle1, handle2, solution, resultConstraints, constraints, 1);

          return true;
        }

      }

    }

  }

  return false;  // Found no conflict so a valid solution is given
}























int mainSearch(FlatlandCBS& f) {

	SearchCBS_t cbs(f);

	bool status = cbs.search(f.solution);

  if (!status) return 1;

  for (size_t handle = 0; handle < f.solution.size(); handle++) {
    std::cout << "Actions for: " << handle << std::endl;
    for (const auto& a : f.solution[handle].actions) {
      std::cout << a.first << ", ";
    }
    std::cout << std::endl << std::endl;
  }

/*
    p::list handleSolution;
    std::cout << "Solution for: " << handle << std::endl;
    for (size_t i = 0; i < f.solution[handle].states.size(); i++) {
      std::cout << f.solution[handle].states[i];

      // False way to print the Actions and State, but it is correct
      int agentSpeed = f.agents[handle].speed;
      if (i % agentSpeed == 0) { 
        std::cout << "->" << f.solution[handle].actions[i / agentSpeed].first << "(cost: " << f.solution[handle].actions[i / agentSpeed].second << ")";
      }
      std::cout << std::endl;
    }
  }
  */

	return 0;
}

p::dict getActions(FlatlandCBS& f) {
  p::dict result;
  for (int handle = 0; (unsigned) handle < f.solution.size(); handle++) {
    p::list tmp;
    for (const auto& a : f.solution[handle].actions) {
      tmp.append(toInt(a.first));
    }
    result[handle] = tmp;
  }
  
  return result;
}

p::dict FlatlandCBS::getStatistic() {
	p::dict result;

	result["Expanded LowLevelNodes"] = nodeExpandCount;
  result["Expanded HighLevelNodes"] = highLevelNodeExpandCount;

	return result;
}



BOOST_PYTHON_MODULE(libFlatlandCBS)
{		
	Py_Initialize();
  np::initialize();
  using namespace boost::python;
    
  class_<FlatlandCBS>("FlatlandCBS", init<object>())
    .def("getStatistic", &FlatlandCBS::getStatistic);
  
  def("mainSearch", &mainSearch);
  def("getActions", &getActions);
}