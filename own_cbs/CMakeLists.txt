cmake_minimum_required (VERSION 3.8)
project (libFlatlandCBS)


# Add all the files to the library so it can get created
ADD_LIBRARY(FlatlandCBS SHARED
				main.cpp
				include/flatland.hpp
				include/a_star.hpp
				include/agent.hpp
				include/cbs.hpp
				include/constraint.hpp
				include/planresult.hpp
				include/neighbor.hpp
				include/main_includes.hpp)


# Set the Flags and the CXX command
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra -fconcepts")


INCLUDE_DIRECTORIES(include)

set(boostPython python)
find_package(PythonInterp 3.6 REQUIRED)
find_package(PythonLibs 3.6 REQUIRED)

include_directories(${PYTHON_INCLUDE_DIRS})

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF)
FIND_PACKAGE(Boost REQUIRED COMPONENTS system program_options numpy ${boostPython})


if(Boost_FOUND)

	include_directories(${Boost_INCLUDE_DIRS})
	target_link_libraries(FlatlandCBS ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})

else()
	message(FATAL_ERROR "Could not find boost.")
endif()