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

typedef AStar<FlatlandCBS, State, Agent <State, GridLocation>, Action> AStar_t;


criticalStateResult FlatlandCBS::getCriticalState(const size_t conflictTime, size_t handle1, size_t handle2, const std::vector<PlanResult <Action, State> >& solution, const int edge) {

  std::pair<bool, State> stateOne = std::make_pair(true, solution[handle1].states[conflictTime]);
  // Take the previous state if there is an edge conflict
  std::pair<bool, State> stateTwo = std::make_pair(true, solution[handle2].states[conflictTime - edge]);

  GridLocation target = m_agents[handle2].targetLocation;

  while (true) {

    // std::cout << stateOne.first << ", " << stateOne.second << std::endl;
    // std::cout << stateTwo.first << ", " << stateTwo.second << std::endl;
    
    // If true the inital state of agent 1 has been reached, now determine if applicable or applicable
    if (stateOne.second == m_agents[handle1].initialState) {
      if (m_possibleActions[GridLocation(stateOne.second.y, stateOne.second.x, stateOne.second.dir)].size() >= 2) return criticalStateResult(Status::InitialApplicable, stateOne.second, stateTwo.second);

      return criticalStateResult(Status::InitialInapplicable, stateOne.second, stateTwo.second);
    }

    // If true the goal of agent 2 has been reached, now determine if applicable or inapplicable
    if (stateTwo.second == target) {
      if (m_possibleActions[GridLocation(stateOne.second.y, stateOne.second.x, stateOne.second.dir)].size() >= 2) return criticalStateResult(Status::GoalApplicable, stateOne.second, stateTwo.second);

      return criticalStateResult(Status::GoalInapplicable, stateOne.second, stateTwo.second);
    }

    if (m_possibleActions[GridLocation(stateOne.second.y, stateOne.second.x, stateOne.second.dir)].size() >= 2) return criticalStateResult(Status::Applicable, stateOne.second, stateTwo.second);

    if (m_possibleActions[GridLocation(stateTwo.second.y, stateTwo.second.x, stateTwo.second.dir)].size() >= 2) {
      // Make sure the critical state is not an at the first step of an edge error
      if (edge != 1 || (unsigned) stateOne.second.time != conflictTime) return criticalStateResult(Status::Inapplicable, stateOne.second, stateTwo.second);
    }

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
  return std::make_pair(false, state);
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


void FlatlandCBS::handleConflicts(const State& conflictState1, const State& conflictState2, const size_t conflictTime, const size_t handle1, const size_t handle2, std::vector<PlanResult <Action, State> >& solution, std::vector<Constraints>& oldConstraints, std::vector<std::pair<size_t, Constraints>>& resultConstraints, std::vector<std::pair< std::pair<size_t, size_t> , std::pair<Constraints, Constraint>>>& resultDoubleConstraints, bool edge) {

  Constraints constraints;

  // Check for rear-end collision
  if (conflictState1.dir == conflictState2.dir && edge == 0) {
    if (DEBUG) std::cout << "Rear-end collision" << std::endl;

    // Note: There is no need to check the previous state, because when an agent has an rear-end collision with an agent
    // which waits at the inital state, the value 0 gets returned
    std::pair<bool, State> prevAgentOne = getPrevState(conflictState1, handle1, solution);

    // Check if agent 1 is already waiting by checking if it was already in the state the timestep before
    if (conflictTime - prevAgentOne.second.time > 1) {
      std::pair<bool, State> nextAgentOne = getNextState(conflictState1, handle1, solution);

      // If the previous state is the default state, extend the initial constraints
      if (solution[handle2].states[conflictTime-1].equalExceptTime(defaultState)) {

        constraints.addInitial(nextAgentOne.second.time-1);
      }
      else constraints.add(Constraint(prevAgentOne.second.time+1, nextAgentOne.second.time-1, conflictState1.y, conflictState1.x));

      resultConstraints.emplace_back(std::make_pair(handle2, constraints));
    }

    else {
      std::pair<bool, State> prevAgentTwo = getPrevState(conflictState2, handle2, solution);
      std::pair<bool, State> nextAgentTwo = getNextState(conflictState2, handle2, solution);

      if (solution[handle1].states[conflictTime-1].equalExceptTime(defaultState)) {

        constraints.addInitial(nextAgentTwo.second.time-1);
      }
      else constraints.add(Constraint(prevAgentTwo.second.time+1, nextAgentTwo.second.time-1, conflictState2.y, conflictState2.x));

      resultConstraints.emplace_back(std::make_pair(handle1, constraints));
    }

    return;
  }
  
  // Check for head collision
  if ((4 + (conflictState1.dir-1)) % 4 == conflictState2.dir || (conflictState1.dir+1) % 4 == conflictState2.dir) {

    State nextAgentOne = getNextState(conflictState1, handle1, solution).second;
    State prevAgentTwo = getPrevState(conflictState2, handle2, solution).second;
    

    if (!nextAgentOne.equalExceptTime(prevAgentTwo)) {
      if (DEBUG) std::cout << "Head collision" << std::endl;

      State prevAgentOne = getPrevState(conflictState1, handle1, solution).second;
      State nextAgentTwo = getNextState(conflictState2, handle2, solution).second;

      // Time in which agent 1 would enter the location
      int lowerBound = prevAgentOne.time + 1;

      // Time in which agent 2 leaved the location
      int upperBound = nextAgentTwo.time - 1;

      // If the previous state is the default state, extend the initial constraints
      if (solution[handle1].states[conflictTime-1].equalExceptTime(defaultState)) {

        constraints.addInitial(upperBound);
      }

      else constraints.add(Constraint(lowerBound, upperBound, conflictState1.y, conflictState1.x));

      resultConstraints.emplace_back(std::make_pair(handle1, constraints));

      return;
    }
  }

  // Normal conflict

  if (edge == 0) {
    State nextAgentOne = getNextState(conflictState1, handle1, solution).second;
    State nextAgentTwo = getNextState(conflictState2, handle2, solution).second;

    if (nextAgentOne.equalExceptTime(nextAgentTwo)) {
      
      State prevAgentOne = getPrevState(conflictState1, handle1, solution).second;

      // TODO: Add the speed and may the goal

      // Time in which agent 1 would enter the location
      int lowerBound = prevAgentOne.time + 1;

      // Time in which agent 2 leaved the location
      int upperBound = nextAgentTwo.time - 1;

      constraints.add(Constraint(lowerBound, upperBound, conflictState1.y, conflictState1.x));

      resultConstraints.emplace_back(std::make_pair(handle1, constraints));

      return;

    }
  }



  criticalStateResult resultOne = getCriticalState(conflictTime, handle1, handle2, solution, edge);
  criticalStateResult resultTwo = getCriticalState(conflictTime, handle2, handle1, solution, edge);

  if (DEBUG) std::cout << "Status: " << resultOne.status << " -> Agent " << handle1 << ", cs: " << resultOne.firstState << ", o: " << resultOne.secondState << std::endl;
  if (DEBUG) std::cout << "Status: " << resultTwo.status << " -> Agent " << handle2 << ", cs: " << resultTwo.firstState << ", o: " << resultTwo.secondState << std::endl;


  // If there is no escape for both agents, constrain the slower agent
  if (resultOne.status == Status::InitialInapplicable && resultTwo.status == Status::InitialInapplicable) {

    if (m_agents[handle1].speed > m_agents[handle2].speed) {
      std::cout << "Agent " << handle1 << " is slower!" << std::endl;

      int upperBound = getNextState(resultOne.secondState, handle2, solution).second.time - 1;

      constraints.addInitial(upperBound); 

      resultConstraints.emplace_back(std::make_pair(handle1, constraints));

    } else {
      if (DEBUG) std::cout << "Agent " << handle2 << " is slower!" << std::endl;

      int upperBound = getNextState(resultTwo.secondState, handle1, solution).second.time - 1;

      constraints.addInitial(upperBound); 

      resultConstraints.emplace_back(std::make_pair(handle2, constraints));
    }

    return;
  }

  
  constrainAgent(resultOne, conflictTime, handle1, handle2, solution, oldConstraints, resultConstraints, resultDoubleConstraints);
  constrainAgent(resultTwo, conflictTime, handle2, handle1, solution, oldConstraints, resultConstraints, resultDoubleConstraints);
  
  // Nur vielleicht, muss evtl. weg

  /*
  else if (resultOne.status == Status::InitialInapplicable) {
    
    constrainAgent(resultTwo, conflictTime, handle2, handle1, solution, oldConstraints, resultConstraints, resultDoubleConstraints);
    
  }


  else if (resultTwo.status == Status::InitialInapplicable) {

    constrainAgent(resultOne, conflictTime, handle1, handle2, solution, oldConstraints, resultConstraints, resultDoubleConstraints);

  }

  else {
    
    constrainAgent(resultOne, conflictTime, handle1, handle2, solution, oldConstraints, resultConstraints, resultDoubleConstraints);
    constrainAgent(resultTwo, conflictTime, handle2, handle1, solution, oldConstraints, resultConstraints, resultDoubleConstraints);

  }
  */
  
}


void FlatlandCBS::constrainAgent(const criticalStateResult& result, const size_t conflictTime, const size_t handle1, const size_t handle2, std::vector<PlanResult <Action, State> >& solution, std::vector<Constraints>& oldConstraints, std::vector<std::pair<size_t, Constraints>>& resultConstraints, std::vector<std::pair< std::pair<size_t, size_t> , std::pair<Constraints, Constraint>>>& resultDoubleConstraints) {
  // TODO: Add the special case, Local search, nochmal drüberschauen

  Constraints constraints;
  
  if (result.status == Status::Applicable) {
    // Lower bound: agent enter the critical state + speed

    // Can always be applied without checking
    int lowerBoundNext = getPrevState(result.firstState, handle1, solution).second.time + 1;
    int lowerBoundCurr = lowerBoundNext + m_agents[handle1].speed;

    int upperBound = 0;

    // Get the corresponding critical state from agent 2
    State currState = getNextState(result.secondState, handle2, solution).second;
    int count = 1;
    while (true) {

      // // std::cout << currState << std::endl;

      // If the found state is the target of agent 2, set to timestep of it
      if (currState == m_agents[handle2].targetLocation) {
        if (DEBUG) std::cout << "Applicable -> targetLocation!" << std::endl;

        upperBound = currState.time;
        upperBound += count * m_agents[handle1].speed;

        break;
      }

      // If the found state is the inital state, local search so agent 1 is taking the other route
      if (currState.y == m_agents[handle1].initialState.y && currState.x == m_agents[handle1].initialState.x) {

        if (DEBUG) std::cout << "Applicable -> initial!" << std::endl;

        int r = initialSearch(result, handle1, oldConstraints, lowerBoundCurr);

        if (r == -1) return;

        upperBound = lowerBoundCurr + r;

        break;
      }

      // If the found state is an applicable state, take the time agent 2 would leave this state
      if (m_possibleActions[GridLocation(currState.y, currState.x, currState.dir)].size() >= 2) {

        if (DEBUG) std::cout << "Applicable -> applicable!" << std::endl;

        // Cannot be the target state, otherwise a if case earlier will trigger
        upperBound = getNextState(currState, handle2, solution).second.time - 1;
        upperBound += count * m_agents[handle1].speed;

        break;
      }

      // Make sure to loop trough the solution from agent 2
      currState = getNextState(currState, handle2, solution).second;
      count++;
    }

    // If this is the target of agent 1, there would just be two constraints on the same location --> do not matter
    State nextState = getNextState(result.firstState, handle1, solution).second;

    constraints.add(Constraint(lowerBoundCurr, upperBound, result.firstState.y, result.firstState.x));
    constraints.add(Constraint(lowerBoundNext, upperBound, nextState.y, nextState.x));

    if ((unsigned)result.firstState.time == conflictTime) {
      if (DEBUG) std::cout << "Double constrain!" << std::endl;
      Constraint conOther = Constraint(lowerBoundNext, upperBound, result.firstState.y, result.firstState.x);
      resultDoubleConstraints.emplace_back(std::make_pair(std::make_pair(handle1, handle2), std::make_pair(constraints, conOther)));
    }
    else resultConstraints.emplace_back(std::make_pair(handle1, constraints));

  }

  if (result.status == Status::Inapplicable) {

    // Lower bound: agent 1 enter the critical state
    int lowerBound = getPrevState(result.firstState, handle1, solution).second.time + 1;

    // Upper bound: agent 2 leaves the critical state
    int upperBound = getNextState(result.secondState, handle2, solution).second.time - 1;

    constraints.add(Constraint(lowerBound, upperBound, result.firstState.y, result.firstState.x));

    resultConstraints.emplace_back(std::make_pair(handle1, constraints));

  }

  if (result.status == Status::InitialInapplicable) {
    // Upper bound: agent 2 leaves the critical state
    int upperBound = getNextState(result.secondState, handle2, solution).second.time - 1;

    constraints.addInitial(upperBound); 

    resultConstraints.emplace_back(std::make_pair(handle1, constraints));
  }

  if (result.status == Status::InitialApplicable) {
    if (DEBUG) std::cout << "Initial Applicable!" << std::endl;


    int lowerBoundNext = 1;
    int lowerBoundCurr = m_agents[handle1].speed;

    int r = initialSearch(result, handle1, oldConstraints, lowerBoundCurr);

    if (r == -1) return;

    int upperBound = lowerBoundCurr + r;

    State nextState = getNextState(result.firstState, handle1, solution).second;

    constraints.add(Constraint(lowerBoundCurr, upperBound, result.firstState.y, result.firstState.x));
    constraints.add(Constraint(lowerBoundNext, upperBound, nextState.y, nextState.x));


    if ((unsigned)result.firstState.time == conflictTime) {
      if (DEBUG) std::cout << "Double constrain!" << std::endl;
      Constraint conOther = Constraint(lowerBoundNext, upperBound, result.firstState.y, result.firstState.x);
      resultDoubleConstraints.emplace_back(std::make_pair(std::make_pair(handle1, handle2), std::make_pair(constraints, conOther)));
    }
    else resultConstraints.emplace_back(std::make_pair(handle1, constraints));
  }

  if (result.status == Status::GoalInapplicable) {
    // Lower bound: timestep where agent would enter goal location
    int lowerBound = getPrevState(result.firstState, handle1, solution).second.time + 1;

    // Uppter bound: timestep of last state in solution of agent 2
    int upperBound = solution[handle2].states.back().time;

    constraints.add(Constraint(lowerBound, upperBound, result.firstState.y, result.firstState.x));

    resultConstraints.emplace_back(std::make_pair(handle1, constraints));

  }

  if (result.status == Status::GoalApplicable) {
    // Lower bound: timestep where agent would enter goal location
    int lowerBoundNext = getPrevState(result.firstState, handle1, solution).second.time + 1;
    int lowerBoundCurr = lowerBoundNext + m_agents[handle1].speed;
    

    // Uppter bound: timestep of last state in solution of agent 2
    int upperBound = solution[handle2].states.back().time;

    // If this is the target of agent 1, there would just be two constraints on the same location --> do not matter
    State nextState = getNextState(result.firstState, handle1, solution).second;
    
    constraints.add(Constraint(lowerBoundNext, upperBound, nextState.y, nextState.x));

    if ((unsigned)result.firstState.time == conflictTime) {
      if (DEBUG) std::cout << "Goall \"double\" constrain!" << std::endl;

      constraints.add(Constraint(lowerBoundNext, upperBound, result.firstState.y, result.firstState.x));

    } else constraints.add(Constraint(lowerBoundCurr, upperBound, result.firstState.y, result.firstState.x));

    resultConstraints.emplace_back(std::make_pair(handle1, constraints));
  }
}


int FlatlandCBS::initialSearch(const criticalStateResult result, const int handle, std::vector<Constraints>& oldConstraints, const int lowerBoundCurr) {
  // Find the new inital state
  State newInitialState;

  for (const auto& gridLoc : m_possibleActions[GridLocation(result.firstState.y, result.firstState.x, result.firstState.dir)]) {
    if (gridLoc.y != result.firstState.y || gridLoc.x != result.firstState.x || gridLoc.dir != result.firstState.dir) {
      newInitialState = State(lowerBoundCurr, gridLoc.y, gridLoc.x, gridLoc.dir);
      break;
      }
  }

  // Save the old values to reset later
  int tmpinitialConstraintEndTime = oldConstraints[handle].initialConstraintEndTime;
  State tmpInitalState = m_agents[handle].initialState;

  // Reset the inital constraints and the initial state
  oldConstraints[handle].initialConstraintEndTime = -1;
  m_agents[handle].initialState = newInitialState;
  
  PlanResult <Action, State> tmp;

  // Check if this state can lead to a node, otherwiese skip
  if (m_agents[handle].getHeuristicValue(newInitialState) != INT_MIN) {

    AStar_t astar(*this, &oldConstraints[handle]);    
    astar.search(m_agents[handle], tmp, true);
  } 
  
  else return -1;

  /*
  for (const auto& s : tmp.states) {
    std::cout << s << std::endl;
  }
  */

  // Set the inital constraints and the initial state back
  oldConstraints[handle].initialConstraintEndTime = tmpinitialConstraintEndTime;
  m_agents[handle].initialState = tmpInitalState;

  return tmp.states.size();
}