#pragma once

#include "problem.h"
#include "model.h"

struct follower_solver : public model_single
{
	using cost_map_type = std::map<problem::edge_descriptor, cost_type>;
	using path = std::vector<int>;

	cost_map_type tolled_cost_map;
	double time;

	follower_solver(const problem& prob);

	std::vector<path> solve(const std::vector<cost_type>& tolls);
};
