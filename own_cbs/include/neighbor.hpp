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

template <typename Action, typename State>
struct Neighbor {
  Neighbor(const State& state, const Action& action, const int cost) : state(state), action(action), cost(cost) {}

  State state;
  Action action;
  int cost;

  friend std::ostream& operator<<(std::ostream& os, const Neighbor& n) {
    return os << "New:" << n.state << ", a=" << n.action << ", c=" << n.cost;
  }
};