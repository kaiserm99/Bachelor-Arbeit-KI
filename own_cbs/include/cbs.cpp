#include <iostream>

#include <main_includes.hpp>
#include <cbs.hpp>


SearchCBS::SearchCBS (FlatlandCBS& flatlandCBS) : m_flatlandCBS(flatlandCBS) {

}

bool SearchCBS::search() {

  std::cout << "Hallo du Opfer" << std::endl;

  GridLocation a = m_flatlandCBS.getGridLocation(1, 1, 1);

  std::cout << a << std::endl;
}