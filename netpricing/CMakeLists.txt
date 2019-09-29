# CMakeList.txt : CMake project for netpricing, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

# Add source to this project's executable.
add_executable(netpricing
	"netpricing.cpp"
	"problem.cpp"
	"models/standard_model.cpp"
	"models/benders_model_original.cpp")

target_link_libraries(netpricing
	-lilocplex
	-lconcert
	-lcplex
	-lpthread
	-ldl)

target_compile_definitions(netpricing PUBLIC -DIL_STD)

# TODO: Add tests and install targets if needed.
