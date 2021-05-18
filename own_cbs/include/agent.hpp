#pragma once

#include <iostream>

#include <boost/python/numpy.hpp>
#include <boost/python.hpp>

namespace p = boost::python;
namespace np = boost::python::numpy;


template <typename State, typename GridLocation>
class Agent {
  public:
    Agent(int handle, State initialState, GridLocation targetLocation, p::object heuristics) : 
                handle(handle),
                initialState(initialState),
                targetLocation(targetLocation),
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
    std::vector<State> solution;
    int soulutionCost;

    friend std::ostream& operator<<(std::ostream& os, const Agent& other) {
      return os << "Agent " << other.handle << " , init=" << other.initialState << ", target=" << other.targetLocation;
    }

  private:
    np::ndarray m_heuristics;
};