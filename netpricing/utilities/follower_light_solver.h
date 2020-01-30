#pragma once

#include "follower_solver_base.h"
#include "../graph/light_graph.h"

struct follower_light_solver : public follower_solver_base
{
	light_graph lgraph;

	follower_light_solver(const problem& prob);

	virtual std::vector<path> solve_impl(const std::vector<cost_type>& tolls) override;
};
