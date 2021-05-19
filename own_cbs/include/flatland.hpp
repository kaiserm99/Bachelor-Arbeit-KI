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

    void onExpandNode();

  private:
    p::object m_railEnv;
    p::object m_rail;
    int m_dimy;
    int m_dimx;

    np::ndarray m_map;
    const Constraints* m_constraints;

    size_t nodeExpandCount;
};