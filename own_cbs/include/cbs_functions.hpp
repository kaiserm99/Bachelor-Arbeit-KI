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


std::pair<int, State> FlatlandCBS::getCriticalState(const State& conflictState1, const State& conflictState2, size_t handle1, size_t handle2, const std::vector<PlanResult <Action, State> >& solution) {
  // first element is the stauts: -1 -> no escape for agent 1, 0 -> state inapplicable, 1 -> state applicable

  std::pair<bool, State> stateOne = std::make_pair(true, conflictState1);
  std::pair<bool, State> stateTwo = std::make_pair(true, conflictState2);

  while (true) {
    
    if (stateOne.first == false) return std::make_pair(-1, defaultState);

    if (stateTwo.first == false) {
      if (possibleActions[GridLocation(stateOne.second.y, stateOne.second.x, stateOne.second.dir)].size() >= 2) return std::make_pair(1, stateOne.second);

      return std::make_pair(0, stateOne.second);
    }

    if (possibleActions[GridLocation(stateOne.second.y, stateOne.second.x, stateOne.second.dir)].size() >= 2) return std::make_pair(1, stateOne.second);

    if (possibleActions[GridLocation(stateTwo.second.y, stateTwo.second.x, stateTwo.second.dir)].size() >= 2) return std::make_pair(0, stateOne.second);

    stateOne = getPrevState(stateOne.second, handle1, solution);
    stateTwo = getNextState(stateTwo.second, handle2, solution);

  }

}

std::pair<bool, State> FlatlandCBS::getNextState(const State& state, const int handle, const std::vector<PlanResult <Action, State> >& solution) {
  /*
  std::cout << getNextState(solution[handle1].states[15], handle1, solution).first <<  ", " << getNextState(solution[handle1].states[15], handle1, solution).second << std::endl;
  std::cout << getNextState(solution[handle1].states[16], handle1, solution).first <<  ", " << getNextState(solution[handle1].states[16], handle1, solution).second << std::endl;
  std::cout << getNextState(solution[handle1].states[17], handle1, solution).first <<  ", " << getNextState(solution[handle1].states[17], handle1, solution).second << std::endl;
  */

  // Start at the State in the next timestep and loop trough it, until there was another position
  size_t currTime;
  for (currTime = state.time+1; currTime < solution[handle].states.size(); currTime++) {

    State currState = solution[handle].states[currTime];

    if (state.y != currState.y || state.x != currState.x) return std::make_pair(true, currState);
  }

  // Return the goal state, because the given state was the second last state
  return std::make_pair(false, State(currTime, agents[handle].targetLocation));
}

std::pair<bool, State> FlatlandCBS::getPrevState(const State& state, const int handle, const std::vector<PlanResult <Action, State> >& solution) {
  /*
  std::cout << getPrevState(solution[handle1].states[0], handle1, solution).first <<  ", " << getPrevState(solution[handle1].states[0], handle1, solution).second << std::endl;
  std::cout << getPrevState(solution[handle1].states[9], handle1, solution).first <<  ", " << getPrevState(solution[handle1].states[9], handle1, solution).second << std::endl;
  std::cout << getPrevState(solution[handle1].states[10], handle1, solution).first <<  ", " << getPrevState(solution[handle1].states[10], handle1, solution).second << std::endl;
  */

  // Start at the State in the previous timestep and loop trough it, until there was another position
  for (int currTime = state.time-1; currTime >= 0; currTime--) {

    State currState = solution[handle].states[currTime];

    if (state.y != currState.y || state.x != currState.x) return std::make_pair(true, currState);
  }

  // Return the initial state, because we have already checked the first state
  return std::make_pair(false, State(0, state));
}

void FlatlandCBS::handleConflicts(const State& conflictState1, const State& conflictState2, const int conflictTime, const size_t handle1, const size_t handle2, std::vector<PlanResult <Action, State> >& solution, std::vector<std::pair<size_t, Constraint>>& resultConstraints, bool edge) {

  if (!edge) std::cout << std::endl << "[t=" << conflictTime << "] Vertex conflict at: " << conflictState1 << ", " << conflictState2 << std::endl;
  else std::cout << std::endl << "[t=" << conflictTime << "] Edge conflict at: " << conflictState1 << ", " << conflictState2 << std::endl;

  std::pair<int, State> resultOne = getCriticalState(conflictState1, conflictState2, handle1, handle2, solution);
  std::pair<int, State> resultTwo = getCriticalState(conflictState2, conflictState1, handle2, handle1, solution);

  std::cout << "Status: " << resultOne.first << " -> Agent " << handle1 << ", critcal state: " << resultOne.second << std::endl;
  std::cout << "Status: " << resultTwo.first << " -> Agent " << handle2 << ", critcal state: " << resultTwo.second << std::endl;

  // Check for rear-end collision
  if (conflictState1.dir == conflictState2.dir) {
    std::cout << "Rear-end collision" << std::endl;

    
    std::pair<bool, State> prevAgentOne = getPrevState(conflictState1, handle1, solution);

    // Check if agent 1 is already waiting by checking if it was already in the state the timestep before
    if (conflictTime - prevAgentOne.second.time > 1) {
      std::pair<bool, State> nextAgentOne = getNextState(conflictState1, handle1, solution);

      resultConstraints.emplace_back(std::make_pair(handle2, Constraint(prevAgentOne.second.time, nextAgentOne.second.time, conflictState1.y, conflictState1.x)));
    }

    else {
      std::pair<bool, State> prevAgentTwo = getPrevState(conflictState2, handle2, solution);
      std::pair<bool, State> nextAgentTwo = getNextState(conflictState2, handle2, solution);

      resultConstraints.emplace_back(std::make_pair(handle1, Constraint(prevAgentTwo.second.time, nextAgentTwo.second.time, conflictState2.y, conflictState2.x)));
    }


    return;
  }
  
}