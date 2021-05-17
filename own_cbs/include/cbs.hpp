#pragma once

#include <main_includes.hpp>

class SearchCBS {
  public:
    SearchCBS(FlatlandCBS& flatlandCBS);
    bool search();

  private:
    FlatlandCBS& m_flatlandCBS;
};
