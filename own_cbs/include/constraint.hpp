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

  Constraint(int startingTime, int endingTime, int y, int x) : startingTime(startingTime), endingTime(endingTime), y(y), x(x)  {}
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
    return os << "C(" << c.startingTime << "-" << c.endingTime << ": y=" << c.y << ", x=" << c.x << ")";
  }
};

// ---------------------- Constraint End -----------------------------------

struct Constraints {

  std::vector<Constraint> constraints;

  // Add a constraint to the list
  void add(const Constraint& other) {
    constraints.emplace_back(other);
  }

  // See: C++ extend a vector with another vector, (https://stackoverflow.com/questions/313432/c-extend-a-vector-with-another-vector)
  void extend(const Constraints& other) {
    constraints.reserve(constraints.size() + std::distance(other.constraints.begin(), other.constraints.end()));
    constraints.insert(constraints.end(), other.constraints.begin(), other.constraints.end());
  }

  // If there is a Constraint where the current state is bewtween the timestamp and the same position, return false
  bool checkState(const State& s, const int speed) const {
    for (const auto& c : constraints) {

      if (s.y != c.y || s.x == c.x) continue;  // If the Constraint and the State has not the same Position --> no need to chekc

      // Check all the States with depending on the speed afterwards
      for (int offset = 0; offset < speed; offset++) {
        if (c.startingTime <= s.time + offset && s.time + offset <= c.endingTime) return false;
      }
    }
    return true;
  }

  friend std::ostream& operator<<(std::ostream& os, const Constraints& c) {
    for (const auto& vc : c.constraints) { os << vc << std::endl; }

    return os;
  }

};