#pragma once

#include "../problem.h"
#include "../model.h"

struct follower_solver_base : public model_single
{
	using path = std::vector<int>;
	double time;

	follower_solver_base(const problem& prob);

	std::vector<path> solve(const std::vector<cost_type>& tolls);
	virtual std::vector<path> solve_impl(const std::vector<cost_type>& tolls) = 0;

	cost_type get_cost(const path& p, const std::vector<cost_type>& tolls) const;
};
