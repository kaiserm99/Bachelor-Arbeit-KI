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

namespace p = boost::python;
namespace np = boost::python::numpy;

class FlatlandCBS {
  public:
    FlatlandCBS(p::object railEnv);
    void setConstraints(const Constraints* constraints);
    void getNeighbors(const State& s, const int speed, std::vector<Neighbor <Action, State> >& neighbors);
    bool checkState(const State& s);
    std::vector<NewGridLocation> getNextGridLocations(const GridLocation& loc, const int direction, std::vector<int>& possibleTransitions);
    GridLocation getGridLocation(const int y, const int x, const int direction);
    std::vector<int> getTransitions(const GridLocation& loc, const int direction);
    bool all(const std::vector<int>& v);

    std::vector<Agent <State, GridLocation> > agents;
    std::unordered_map<GridLocation, std::vector<NewGridLocation>> possibleActions;

    // Methodes for the CBS search
    bool getFirstConflict(std::vector<PlanResult <Action, State> >& solution, std::vector<std::pair<size_t, Constraints>>& resultConstraints, std::vector<Constraints>& constraints);
    void handleConflicts(const State& conflictState1, const State& conflictState2, const int errorTime, const size_t handle1, const size_t handle2, std::vector<PlanResult <Action, State> >& solution, std::vector<std::pair<size_t, Constraints>>& resultConstraints, std::vector<Constraints>& constraints, bool edge);
    State getCriticalState(int conflictTime, size_t handle1, size_t handle2, const std::vector<PlanResult <Action, State> >& solution);


    // Statistics methodes
    void onExpandNode();
    void onExpandHighLevelNode();
    p::dict getStatistic();

    // The final actions get saved here
    std::vector<PlanResult<Action, State> > solution;

    

  private:
    p::object m_railEnv;
    p::object m_rail;
    int m_dimy;
    int m_dimx;

    np::ndarray m_map;
    const Constraints* m_constraints;

    // Statistic variables
    size_t nodeExpandCount = 0;
    size_t highLevelNodeExpandCount = 0;
};