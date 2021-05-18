#include <fstream>
#include <iostream>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <boost/python/numpy.hpp>
#include <boost/python.hpp>

namespace p = boost::python;
namespace np = boost::python::numpy;

#include <main_includes.hpp>
#include <cbs.hpp>
#include <agent.hpp>
#include <flatland.hpp>




std::ostream& operator<<(std::ostream& os, const Action& a) {
  switch (a) {
    case Action::Up:    os << "2"; break;
    case Action::Left:  os << "1"; break;
    case Action::Right: os << "3"; break;
    case Action::Wait:  os << "4"; break;
  }
  return os;
}


typedef Neighbor <Action> Neighbor_t;
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

    // Make sure to insert the newly created Agent into the handeled list
    Agent<State, GridLocation> agent(handle, initialState, targetLocation, m_map[handle]);
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


void FlatlandCBS::getNeighbors(const State& s, std::vector<Neighbor_t>& neighbors) {

	neighbors.clear();

	// Always append the waiting state
	State n(s.time + 1, s.y, s.x, s.dir);

  neighbors.emplace_back(Neighbor_t(n, Action::Wait, 1));
 
  const auto& next_actions = possibleActions[GridLocation(s.y, s.x, s.dir)];


  for (const auto& next_action : next_actions) {

    // Create the new State, which is checked in the following
    State n(s.time + 1, next_action.y, next_action.x, next_action.dir);

    switch (next_action.action) {
      case 1: neighbors.emplace_back(Neighbor_t(n, Action::Left, 1)); break;

      case 2: neighbors.emplace_back(Neighbor_t(n, Action::Up, 1)); break;

      case 3: neighbors.emplace_back(Neighbor_t(n, Action::Right, 1)); break;
    }
  }
}


std::vector<NewGridLocation> FlatlandCBS::getNextGridLocations(const GridLocation& loc, const int& direction, std::vector<int>& possibleTransitions) {
		
	std::vector<NewGridLocation> result;

	std::vector<int> directions{(4 + (direction-1) % 4) % 4, direction, (direction+1) % 4, (direction+2) % 4};
	std::vector<int> actions{1, 2, 3, 2};

	for (int dir = 0; dir < 4; dir++) {

		if (possibleTransitions[directions[dir]]) {

			const GridLocation acc = getGridLocation(loc.y, loc.x, directions[dir]);

			result.emplace_back(NewGridLocation(acc.y, acc.x, directions[dir], actions[dir]));
		}

	}

	return result;
}


GridLocation FlatlandCBS::getGridLocation(const int& y, const int& x, const int& direction) {
	switch (direction) {
		case 0: return GridLocation(y-1, x  ); break;
	  case 1: return GridLocation(y  , x+1); break;
	  case 2: return GridLocation(y+1, x  ); break;
		case 3: return GridLocation(y  , x-1); break;
	}

	assert(0);
}

std::vector<int> FlatlandCBS::getTransitions(const GridLocation& loc, const int& direction) {

	std::vector<int> result(4);

	p::tuple possible_transitions = p::extract<p::tuple>(m_rail.attr("get_transitions")(loc.y, loc.x, direction));

	for (int dir = 0; dir < 4; dir++) {
	  const char* currentValue = p::extract<char const *>(p::str(possible_transitions[dir]));
		result[dir] = boost::lexical_cast<int>(currentValue);
	}

	return result;
}

bool FlatlandCBS::all(const std::vector<int>& v) {
	return std::all_of(v.begin(), v.end(), [](int i) { return i==0; });
}

void FlatlandCBS::onExpandNode() {
  nodeExpandCount++;
}





int mainSearch(FlatlandCBS& f) {

	SearchCBS_t cbs(f);
	cbs.search();

	return 0;
}





BOOST_PYTHON_MODULE(libFlatlandCBS)
{		
	Py_Initialize();
  np::initialize();
  using namespace boost::python;
    
    
  class_<FlatlandCBS>("FlatlandCBS", init<object>());
	

	def("mainSearch", &mainSearch);
}