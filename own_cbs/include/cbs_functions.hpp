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


criticalStateResult FlatlandCBS::getCriticalState(const size_t conflictTime, size_t handle1, size_t handle2, const std::vector<PlanResult <Action, State> >& solution, const int edge) {

  std::pair<bool, State> stateOne = std::make_pair(true, solution[handle1].states[conflictTime]);
  // Take the previous state if there is an edge conflict
  std::pair<bool, State> stateTwo = std::make_pair(true, solution[handle2].states[conflictTime - edge]);

  GridLocation target = agents[handle2].targetLocation;

  while (true) {
    
    // If true the inital state of agent 1 has been reached, now determine if applicable or applicable
    if (stateOne.first == false) {
      if (possibleActions[GridLocation(stateOne.second.y, stateOne.second.x, stateOne.second.dir)].size() >= 2) return criticalStateResult(Status::InitialApplicable, defaultState, stateTwo.second);

      return criticalStateResult(Status::InitialInapplicable, defaultState, stateTwo.second);
    }

    // If true the goal of agent 2 has been reached, now determine if applicable or inapplicable
    if (stateTwo.second == target) {
      if (possibleActions[GridLocation(stateOne.second.y, stateOne.second.x, stateOne.second.dir)].size() >= 2) return criticalStateResult(Status::GoalApplicable, stateOne.second, stateTwo.second);

      return criticalStateResult(Status::GoalInapplicable, stateOne.second, stateTwo.second);
    }

    if (possibleActions[GridLocation(stateOne.second.y, stateOne.second.x, stateOne.second.dir)].size() >= 2) return criticalStateResult(Status::Applicable, stateOne.second, stateTwo.second);

    if (possibleActions[GridLocation(stateTwo.second.y, stateTwo.second.x, stateTwo.second.dir)].size() >= 2) return criticalStateResult(Status::Inapplicable, stateOne.second, stateTwo.second);

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

    if ((state.y != currState.y || state.x != currState.x) && currState.time != defaultState.time) return std::make_pair(true, currState);
  }

  // Return the initial state, because we have already checked the first state
  return std::make_pair(false, State(0, state));
}

void FlatlandCBS::handleConflicts(const State& conflictState1, const State& conflictState2, const size_t conflictTime, const size_t handle1, const size_t handle2, std::vector<PlanResult <Action, State> >& solution, std::vector<std::pair<size_t, Constraint>>& resultConstraints, bool edge) {

  // Check for rear-end collision
  if (conflictState1.dir == conflictState2.dir) {
    std::cout << "Rear-end collision" << std::endl;

    // Note: There is no need to check the previous state, because when an agent has an rear-end collision with an agent
    // which waits at the inital state, the value 0 gets returned
    std::pair<bool, State> prevAgentOne = getPrevState(conflictState1, handle1, solution);

    // Check if agent 1 is already waiting by checking if it was already in the state the timestep before
    if (conflictTime - prevAgentOne.second.time > 1) {
      std::pair<bool, State> nextAgentOne = getNextState(conflictState1, handle1, solution);

      resultConstraints.emplace_back(std::make_pair(handle2, Constraint(prevAgentOne.second.time+1, nextAgentOne.second.time-1, conflictState1.y, conflictState1.x)));
    }

    else {
      std::pair<bool, State> prevAgentTwo = getPrevState(conflictState2, handle2, solution);
      std::pair<bool, State> nextAgentTwo = getNextState(conflictState2, handle2, solution);

      resultConstraints.emplace_back(std::make_pair(handle1, Constraint(prevAgentTwo.second.time+1, nextAgentTwo.second.time-1, conflictState2.y, conflictState2.x)));
    }

    return;
  }
  
  // Check for head collision
  if ((4 + (conflictState1.dir-1)) % 4 == conflictState2.dir || (conflictState1.dir+1) % 4 == conflictState2.dir) {
    std::cout << "Head collision" << std::endl;

    // TODO: Implement Logic and constrain the agents based on their speed

    // return;
  }


  // Normal conflict
  criticalStateResult resultOne = getCriticalState(conflictTime, handle1, handle2, solution, edge);
  criticalStateResult resultTwo = getCriticalState(conflictTime, handle2, handle1, solution, edge);

  std::cout << "Status: " << resultOne.status << " -> Agent " << handle1 << ", cs: " << resultOne.firstState << ", o: " << resultOne.secondState << std::endl;
  std::cout << "Status: " << resultTwo.status << " -> Agent " << handle2 << ", cs: " << resultTwo.firstState << ", o: " << resultTwo.secondState << std::endl;


  // If there is no escape for both agents, constrain the slower agent
  if (resultOne.status == Status::InitialInapplicable && resultTwo.status == Status::InitialInapplicable) {

    if (agents[handle1].speed > agents[handle2].speed) {
      std::cout << "Agent " << handle1 << " is slower!" << std::endl;

    } else {
      std::cout << "Agent " << handle2 << " is slower!" << std::endl;
    }

    return;
  }


  if (resultOne.status == Status::Applicable) {

  }

}