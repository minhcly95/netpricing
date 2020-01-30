#pragma once

#include "follower_solver_base.h"

struct follower_solver : public follower_solver_base
{
	using cost_map_type = std::map<problem::edge_descriptor, cost_type>;
	cost_map_type tolled_cost_map;

	follower_solver(const problem& prob);

	virtual std::vector<path> solve_impl(const std::vector<cost_type>& tolls) override;
};
