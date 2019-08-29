# CMakeList.txt : CMake project for netpricing, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

# Add source to this project's executable.
add_executable (netpricing
	"netpricing.cpp"
	"netpricing.h"
	"problem.cpp"
	"problem.h"
	"problem_generator.h"
	"graph_algorithm.h")

# TODO: Add tests and install targets if needed.
