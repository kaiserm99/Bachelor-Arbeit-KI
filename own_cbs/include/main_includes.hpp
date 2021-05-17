#pragma once

#include <unordered_map>

#include <boost/python/numpy.hpp>
#include <boost/python.hpp>

namespace p = boost::python;
namespace np = boost::python::numpy;


enum class Action {
  Up,
  Left,
  Right,
  Wait,
};


struct GridLocation {
  GridLocation(int y, int x, int dir = -1) : y(y), x(x), dir(dir) {}
  int y;
  int x;
  int dir;

  bool operator==(const GridLocation& other) const {
    return std::tie(x, y, dir) == std::tie(other.x, other.y, other.dir);
  }

  friend std::ostream& operator<<(std::ostream& os, const GridLocation& c) {
    return os << "(" << c.y << "," << c.x << ")";    
  }
};

namespace std {
	template <>
	struct hash<GridLocation> {
	  size_t operator()(const GridLocation& s) const {
	    size_t seed = 0;
	    boost::hash_combine(seed, s.y);
	    boost::hash_combine(seed, s.x);
      boost::hash_combine(seed, s.dir);
	    return seed;
	  }
	};
}



struct NewGridLocation {
  NewGridLocation(int y, int x, int dir, int action) : y(y), x(x), dir(dir), action(action) {}
  int y;
  int x;
  int dir;
  int action;

  friend std::ostream& operator<<(std::ostream& os, const NewGridLocation& c) {
    return os << "(y:" << c.y << ", x:" << c.x << ", dir:" << c.dir << ", a:" << c.action << ")";    
  }
};



struct State {
  State(int time, int y, int x, int dir) : time(time), y(y), x(x), dir(dir) {}

  int time;
  int y;
  int x;
  int dir;


  bool operator==(const State& s) const {
    return time == s.time && x == s.x && y == s.y;
  }

  bool equalExceptTime(const State& s) const { return x == s.x && y == s.y; }

  friend std::ostream& operator<<(std::ostream& os, const State& s) {
    return os << s.time << ": (" << s.y << "," << s.x << "," << s.dir << ")";
  }
};

namespace std {
  template <>
  struct hash<State> {
    size_t operator()(const State& s) const {
      size_t seed = 0;
      boost::hash_combine(seed, s.time);
      boost::hash_combine(seed, s.y);
      boost::hash_combine(seed, s.x);
      boost::hash_combine(seed, s.dir);
      return seed;
    }
  };
}

struct Neighbor {
  Neighbor(const State& state, const Action& action, const int cost) : state(state), action(action), cost(cost) {}

  State state;
  Action action;
  int cost;
};


class FlatlandCBS {
  public:
    FlatlandCBS(p::object railEnv);
    void getNeighbors(const State& s, std::vector<Neighbor>& neighbors);
    std::vector<NewGridLocation> getNextGridLocations(const GridLocation& loc, const int& direction, std::vector<int>& possibleTransitions);
    GridLocation getGridLocation(const int& y, const int& x, const int& direction);
    std::vector<int> getTransitions(const GridLocation& loc, const int& direction);
    bool all(const std::vector<int>& v);
    void search();

  private:
    p::object m_railEnv;
    p::object m_rail;
    int m_dimy;
    int m_dimx;

    std::vector<State> m_start;
    std::vector<GridLocation> m_goals;

    std::unordered_map<GridLocation, std::vector<NewGridLocation>> m_edges;

    np::ndarray m_map;
};