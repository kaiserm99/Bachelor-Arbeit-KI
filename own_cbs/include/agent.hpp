#pragma once

namespace p = boost::python;
namespace np = boost::python::numpy;


template <typename State, typename GridLocation>
class Agent {
  public:
    Agent(int handle, State& initialState, GridLocation& targetLocation, int speed, p::object heuristics) : 
                handle(handle),
                initialState(initialState),
                targetLocation(targetLocation),
                speed(speed),
                m_heuristics(p::extract<np::ndarray>(heuristics)) {}


    int getHeuristicValue(const State& state) {
      return (int) boost::lexical_cast<float>(p::extract<char const *>(p::str(m_heuristics[state.y][state.x][state.dir])));
    }

    int getHeuristicValue(int y, int x, int dir) {
      return (int) boost::lexical_cast<float>(p::extract<char const *>(p::str(m_heuristics[y][x][dir])));
    }
    
    bool isSolution(State& state) {
      return state.y == targetLocation.y && state.x == targetLocation.x;
    }

    int handle;
    State initialState;
    GridLocation targetLocation;
    int speed;

    friend std::ostream& operator<<(std::ostream& os, const Agent& other) {
      return os << "Agent " << other.handle << ", start=(" << other.initialState.y << "," << other.initialState.x << "), target=" << other.targetLocation;
    }

  private:
    np::ndarray m_heuristics;
};