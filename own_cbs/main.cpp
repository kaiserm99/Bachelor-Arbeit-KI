#include <fstream>
#include <iostream>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

#include <boost/lexical_cast.hpp>
#include <boost/python/numpy.hpp>
#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>
#include <boost/python.hpp>

#include <main_includes.hpp>
#include <cbs.hpp>

namespace p = boost::python;
namespace np = boost::python::numpy;


std::ostream& operator<<(std::ostream& os, const Action& a) {
  switch (a) {
    case Action::Up:    os << "2"; break;
    case Action::Left:  os << "1"; break;
    case Action::Right: os << "3"; break;
    case Action::Wait:  os << "4"; break;
  }
  return os;
}



FlatlandCBS::FlatlandCBS(p::object railEnv) : m_railEnv(railEnv), m_map(p::extract<np::ndarray>(railEnv.attr("distance_map").attr("get")()))
  {

	// Get the Rail system of the current Map
	m_rail = p::extract<p::object>(railEnv.attr("rail"));

	// Get the Dimensions of the current Map
	m_dimy = p::extract<int>(railEnv.attr("height"));
	m_dimx = p::extract<int>(railEnv.attr("width"));

	std::cout << "Loading Map with (" << m_dimy << " x " << m_dimx << ")" << std::endl;


	// Get the List with all the agents
	p::list agents = p::extract<p::list>(railEnv.attr("agents"));

	for (int handle = 0; handle < len(agents); handle++) {
        
	  // Set the Staring State for every Agent
	  p::tuple start = p::extract<p::tuple>(agents[handle].attr("initial_position"));
	  int dir = p::extract<int>(agents[handle].attr("initial_direction"));
	  m_start.emplace_back(State(0, p::extract<int>(start[0]), p::extract<int>(start[1]), dir));

	  // Set the targets for every Agent
		p::tuple target = p::extract<p::tuple>(agents[handle].attr("target"));
	  m_goals.emplace_back(GridLocation(p::extract<int>(target[0]), p::extract<int>(target[1])));
        
	}


	
	for (int handle = 0; handle < len(agents); handle++) {
		  	
		for (int y = 0; y < m_dimy; y++) {

		  for (int x = 0; x < m_dimx; x++) {

		  	for (int dir = 0; dir < 4; dir++) {

		  		std::cout << p::extract<char const *>(p::str(m_map[handle][y][x][dir])) << std::endl;

		  	}
			}
		}
	}


	// Get all the possible actions in the Environment, based on the state and the direction
	std::vector<int> possibleTransitions;
	for (int y = 0; y < m_dimy; y++) {
		for (int x = 0; x < m_dimx; x++) {
	 		for (int dir = 0; dir < 4; dir++) {
		  	possibleTransitions = getTransitions(GridLocation(y, x), dir);
				
				if (all(possibleTransitions)) { continue; }

				m_edges[GridLocation(y, x, dir)] = getNextGridLocations(GridLocation(y, x), dir, possibleTransitions);
	 		}
		}
	}
}


void FlatlandCBS::getNeighbors(const State& s, std::vector<Neighbor>& neighbors) {

	neighbors.clear();

	// Always append the waiting state
	State n(s.time + 1, s.y, s.x, s.dir);

  neighbors.emplace_back(Neighbor(n, Action::Wait, 1));
 
  const auto& next_actions = m_edges[GridLocation(s.y, s.x, s.dir)];


  for (const auto& next_action : next_actions) {

    // Create the new State, which is checked in the following
    State n(s.time + 1, next_action.y, next_action.x, next_action.dir);

    switch (next_action.action) {
      case 1: neighbors.emplace_back(Neighbor(n, Action::Left, 1)); break;

      case 2: neighbors.emplace_back(Neighbor(n, Action::Up, 1)); break;

      case 3: neighbors.emplace_back(Neighbor(n, Action::Right, 1)); break;
    }
  }
}


std::vector<NewGridLocation> FlatlandCBS::getNextGridLocations(const GridLocation& loc, const int& direction, std::vector<int>& possibleTransitions) {
		
	std::vector<NewGridLocation> result;

	std::vector<int> directions{(direction-1) % 4, direction, (direction+1) % 4, (direction+2) % 4};
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


void FlatlandCBS::search() {
	for (const auto& c : m_goals) {
		std::cout << c << std::endl;
	}
}



int mainSearch(FlatlandCBS& c) {

	std::cout << "Test" << std::endl;

	SearchCBS cbs(c);
	cbs.search();
}





BOOST_PYTHON_MODULE(libFlatlandCBS)
{		
	Py_Initialize();
  np::initialize();
  using namespace boost::python;
    
    
  class_<FlatlandCBS>("FlatlandCBS", init<object>())
  	.def("search", &FlatlandCBS::search)
	;
	

	def("mainSearch", &mainSearch);
}