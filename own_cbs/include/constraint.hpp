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

#include <boost/functional/hash.hpp>


struct Constraint {

  Constraint() {
    startingTime = -1;
    endingTime = -1;
    y = -1;
    x = -1;
  }

  Constraint(int st, int et, int y, int x) :  startingTime(st), endingTime(et), y(y), x(x)  {
    if (st > et) startingTime = et;

    else startingTime = st;

    // assert(startingTime <= endingTime);
  }

  int startingTime;
  int endingTime;
  int y;
  int x;

  bool operator<(const Constraint& other) const {
    return std::tie(startingTime, endingTime, y, x) < std::tie(other.startingTime, other.endingTime, other.y, other.x);
  }

  bool operator==(const Constraint& other) const {
    return std::tie(startingTime, endingTime, y, x) == std::tie(other.startingTime, other.endingTime, other.y, other.x);
  }

  friend std::ostream& operator<<(std::ostream& os, const Constraint& c) {
    if (c.startingTime > 0) return os << "C(" << c.startingTime << "->" << c.endingTime << ": y=" << c.y << ", x=" << c.x << ")";

    return os << "I(" << c.startingTime << "->" << c.endingTime << ": y=" << c.y << ", x=" << c.x << ")";
  }
};

// ---------------------- Constraint End -----------------------------------

struct Constraints {

  std::vector<Constraint> constraints;
  int initialConstraintEndTime = -1;

  // Add a constraint to the list
  void add(const Constraint& other) {
    constraints.emplace_back(other);
  }

  void addInitial(const int endingTime) {
    if (initialConstraintEndTime < endingTime) {
      initialConstraintEndTime = endingTime;
    }
  }

  
  void extend(const Constraints& other) {

    // See: C++ extend a vector with another vector, (https://stackoverflow.com/questions/313432/c-extend-a-vector-with-another-vector)
    if (other.initialConstraintEndTime == -1) {
      constraints.reserve(constraints.size() + std::distance(other.constraints.begin(), other.constraints.end()));
      constraints.insert(constraints.end(), other.constraints.begin(), other.constraints.end());
    } else {
      addInitial(other.initialConstraintEndTime);
    }
  }

  bool checkInitial() const {
    return initialConstraintEndTime != -1;
  }

  // If there is a Constraint where the current state is bewtween the timestamp and the same position, return false
  bool checkState(const State& s, const int speed) const {
    for (const auto& c : constraints) {

      
      if (s.y != c.y || s.x != c.x) continue;  // If the Constraint and the State has not the same Position --> no need to check

      // Check all the States with depending on the speed afterwards
      for (int offset = 0; offset < speed; offset++) {
        if (c.startingTime <= s.time + offset && s.time + offset <= c.endingTime) return false;
      }
    }
    return true;
  }

  friend std::ostream& operator<<(std::ostream& os, const Constraints& c) {
    if (c.checkInitial()) os << "I(0->" << c.initialConstraintEndTime << ")" << std::endl;

    for (const auto& vc : c.constraints) { os << vc << std::endl; }

    return os;
  }

};
