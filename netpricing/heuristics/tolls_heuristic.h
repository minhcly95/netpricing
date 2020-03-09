#pragma once

#include "../problem.h"
#include "../solution.h"
#include "../utilities/follower_light_solver.h"
#include "../utilities/inverse_solver.h"

struct tolls_heuristic
{
	using path = std::vector<int>;
	double time;

	follower_light_solver solver_f;
	inverse_solver solver_i;

	tolls_heuristic(IloEnv& env, const problem& prob);

	solution solve(const std::vector<cost_type>& tolls);
	virtual solution solve_impl(const std::vector<cost_type>& tolls);
};
