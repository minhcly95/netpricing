# CMakeList.txt : CMake project for netpricing, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

find_package(Boost REQUIRED COMPONENTS program_options)
include_directories(${Boost_INCLUDE_DIRS})

# Add source to this project's executable.
add_executable(netpricing
	"netpricing.cpp"
	"netpricing_options.cpp"
	"problem.cpp"
	"problem_multi.cpp"
	"problem_generator.cpp"
	"solution.cpp"
	"shullpro/s_hull_pro.cpp"

	"models/model_utils.cpp"
	"models/standard_model.cpp"
	"models/standard_goal_model.cpp"
	"models/standard_cscut_model.cpp"
	"models/standard_vfcut_model.cpp"
	"models/standard_model_multi.cpp"
	"models/benders_model_original.cpp"
	"models/benders_model_reduced.cpp"
	"models/benders_model_reduced2.cpp"
	"models/benders_xt_model.cpp"
	"models/benders_xy_model.cpp"
	"models/benders_xyt_model.cpp"
	"models/slackbranch_model.cpp"
	"models/value_func_model.cpp"
	
	"csenum/csenum.cpp"
	"csenum/csenum_node.cpp"
	"csenum/csenum_solver.cpp"
	
	"utilities/follower_solver_base.cpp"
	"utilities/follower_solver.cpp"
	"utilities/follower_cplex_solver.cpp"
	"utilities/follower_light_solver.cpp"

	"routines/follower_solver_perftest.cpp"

	"graph/light_graph.cpp")

target_link_libraries(netpricing
	-lilocplex
	-lconcert
	-lcplex
	-lpthread
	-ldl
	${Boost_PROGRAM_OPTIONS_LIBRARY})

target_compile_definitions(netpricing PUBLIC -DIL_STD)
target_compile_options(netpricing PRIVATE -Wno-ignored-attributes)

# TODO: Add tests and install targets if needed.
