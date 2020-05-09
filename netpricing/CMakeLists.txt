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
	
	"models/model_cplex.cpp"
	"models/model_utils.cpp"
	"models/standard_model.cpp"
	"models/standard_goal_model.cpp"
	"models/standard_cscut_model.cpp"
	"models/standard_vfcut_model.cpp"
	"models/light_vfcut_model.cpp"
	"models/standard_model_multi.cpp"
	"models/benders_model_original.cpp"
	"models/benders_model_reduced.cpp"
	"models/benders_model_reduced2.cpp"
	"models/benders_xt_model.cpp"
	"models/benders_xy_model.cpp"
	"models/benders_xyt_model.cpp"
	"models/slackbranch_model.cpp"
	"models/value_func_model.cpp"
	"models/compslack_model.cpp"

	"hybrid/base/hybrid_model.cpp"
	"hybrid/base/formulation.cpp"
	"hybrid/base/preprocessor.cpp"
	"hybrid/base/processed_var_name.cpp"
	"hybrid/formulations/processed_formulation.cpp"
	"hybrid/formulations/standard_formulation.cpp"
	"hybrid/formulations/value_func_formulation.cpp"
	"hybrid/formulations/path_formulation.cpp"
	"hybrid/formulations/vfpath_formulation.cpp"
	"hybrid/formulations/path_didi_formulation.cpp"
	"hybrid/preprocessors/path_preprocessor.cpp"
	"hybrid/preprocessors/spgm_preprocessor.cpp"

	"hybrid/standard_hmodel.cpp"
	"hybrid/value_func_hmodel.cpp"
	"hybrid/path_hmodel.cpp"
	"hybrid/vfpath_hmodel.cpp"
	"hybrid/path_didi_hmodel.cpp"
	"hybrid/filtered_hmodel.cpp"
	"hybrid/processed_hmodel.cpp"
	"hybrid/spgm_hmodel.cpp"
	"hybrid/processed_spgm_hmodel.cpp"

	"heuristics/tolls_heuristic.cpp"
	
	"csenum/csenum.cpp"
	"csenum/csenum_excl.cpp"
	"csenum/csenum_context.cpp"
	"csenum/csenum_node.cpp"
	"csenum/csenum_solver_base.cpp"
	"csenum/csenum_solver_dual_only.cpp"
	"csenum/csenum_solver.cpp"
	"csenum/csenum_solver_excl.cpp"
	
	"utilities/follower_solver_base.cpp"
	"utilities/follower_solver.cpp"
	"utilities/follower_cplex_solver.cpp"
	"utilities/follower_light_solver.cpp"
	"utilities/inverse_solver.cpp"
	"utilities/vfcut_builder.cpp"
	"utilities/set_var_name.cpp"
	"utilities/cplex_compare.cpp"

	"routines/follower_solver_perftest.cpp"
	"routines/inverse_solver_test.cpp"
	"routines/tolls_heuristic_test.cpp"
	"routines/light_graph_test.cpp"

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
