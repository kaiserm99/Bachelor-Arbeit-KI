/* Copyright 2021, University of Freiburg
 * Bachelorarbeit - Foundations of Artificial Intelligence
 *
 * Author: Marco Kaiser <kaiserm@informatik.uni-freiburg.de>
 *
 * Description: 
 *
 * Usage: 
 */

#pragma once

enum class Action {
  Forward,
  Left,
  Right,
  Wait,
  Nothing
};

std::ostream& operator<<(std::ostream& os, const Action& a) {
  switch (a) {
    case Action::Forward:  os << "2"; break;
    case Action::Left:     os << "1"; break;
    case Action::Right:    os << "3"; break;
    case Action::Wait:     os << "4"; break;
    case Action::Nothing:  os << "0"; break;
  }
  return os;
}

int toInt(const Action& a) {
  switch (a) {
    case Action::Forward:  return 2;
    case Action::Left:     return 1;
    case Action::Right:    return 3;
    case Action::Wait:     return 4;
    case Action::Nothing:  return 0;
  }
  assert(0);
}

// ---------------------- Action End -----------------------------------


enum class Status {
  InitialInapplicable,
  InitialApplicable,
  Applicable,
  Inapplicable,
  GoalApplicable,
  GoalInapplicable
};

std::ostream& operator<<(std::ostream& os, const Status& a) {
  switch (a) {
    case Status::InitialInapplicable:   os << "InitialInapplicable"; break;
    case Status::InitialApplicable:     os << "InitialApplicable"; break;
    case Status::Applicable:            os << "Applicable"; break;
    case Status::Inapplicable:          os << "Inapplicable"; break;
    case Status::GoalApplicable:        os << "GoalApplicable"; break;
    case Status::GoalInapplicable:      os << "GoalInapplicable"; break;
  }
  return os;
}


// ---------------------- Status End -----------------------------------

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

// ---------------------- GridLocation End -----------------------------------

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

// ---------------------- NewGridLocation End -----------------------------------

struct State {
  State() {
    time = -1;
    y = -1;
    x = -1;
    dir = -1;
  }

  State(int time, int y, int x, int dir) : time(time), y(y), x(x), dir(dir) {}

  State(int time, GridLocation loc, int dir = -1) : time(time), y(loc.y), x(loc.x), dir(dir) {}

  State(int time, State s) : time(time), y(s.y), x(s.x), dir(s.dir) {}

  int time;
  int y;
  int x;
  int dir;

  State setTime(const int t) const { return State(t, y, x, dir); }

  State timePlusT(const int t) const { return State(time+t, y, x, dir); }

  bool operator==(const State& s) const {
    return time == s.time && y == s.y && x == s.x;
  }

  bool operator==(const GridLocation& l) const {
    return y == l.y && x == l.x;
  }

  bool equalExceptTime(const State& s) const { return y == s.y && x == s.x; }

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

State defaultState = State(-1, -1, -1, -1);
// ---------------------- State End -----------------------------------


struct criticalStateResult {
  criticalStateResult(Status status, State firstState, State secondState) : status(status), firstState(firstState), secondState(secondState) {}

  Status status;
  State firstState;
  State secondState;
};

// ---------------------- criticalStateResult End -----------------------------------
