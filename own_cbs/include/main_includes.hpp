#pragma once

#include <unordered_map>
#include <boost/functional/hash.hpp>


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

template <typename Action>
struct Neighbor {
  Neighbor(const State& state, const Action& action, const int cost) : state(state), action(action), cost(cost) {}

  State state;
  Action action;
  int cost;

  friend std::ostream& operator<<(std::ostream& os, const Neighbor& n) {
    return os << "New:" << n.state << ", a=" << n.action << ", c=" << n.cost;
  }
};