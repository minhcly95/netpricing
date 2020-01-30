#include "follower_solver_base.h"

#include <chrono>

#include "../macros.h"

using namespace std;

follower_solver_base::follower_solver_base(const problem& prob) :
	model_single(prob), time(0) { }

std::vector<follower_solver_base::path> follower_solver_base::solve(const std::vector<cost_type>& tolls)
{
	auto start = chrono::high_resolution_clock::now();

	auto paths = solve_impl(tolls);

	auto end = chrono::high_resolution_clock::now();
	time += chrono::duration<double>(end - start).count();

	return paths;
}

cost_type follower_solver_base::get_cost(const path& p, const std::vector<cost_type>& tolls) const
{
	cost_type cost = 0;

	for (int i = 0; i < p.size() - 1; ++i) {
		auto edge = EDGE_FROM_SRC_DST(prob, p[i], p[i + 1]);
		cost += prob.cost_map[edge];

		if (prob.is_tolled_map[edge]) {
			cost += tolls[EDGE_TO_A1(prob, edge)];
		}
	}

	return cost;
}


