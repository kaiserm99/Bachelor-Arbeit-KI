cmake_minimum_required(VERSION 3.5)

project(libMultiRobotPlanning)

find_package(Boost 1.58 REQUIRED COMPONENTS program_options)
find_package(PkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)


# clang-format
set(ALL_SOURCE_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/cbs.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/timer.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/libMultiRobotPlanning/cbs.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/libMultiRobotPlanning/neighbor.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/libMultiRobotPlanning/planresult.hpp
)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra -fconcepts")

include_directories(
  include
)


## cbs
add_executable(cbs
  cbs.cpp
)
target_link_libraries(cbs
  ${Boost_LIBRARIES}
  yaml-cpp
)
