#pragma once

template <typename Action, typename State>
struct PlanResult {
  std::vector<State> states;
  // Make sure to include the speed of the Agent in the actions
  std::vector<std::pair <Action, int> > actions;

  int cost;
};