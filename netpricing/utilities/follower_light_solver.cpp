#include "follower_light_solver.h"
#include "../macros.h"

#include <algorithm>

using namespace std;
using namespace boost;

follower_light_solver::follower_light_solver(const problem& prob) :
	follower_solver_base(prob), lgraph(prob.graph) { }

vector<follower_light_solver::path> follower_light_solver::solve_impl(const vector<cost_type>& tolls)
{
	// Set toll to graph
	LOOP(a, A1) {
		SRC_DST_FROM_A1(prob, a);
		lgraph.edge(src, dst).toll = tolls[a] * 0.9999;		// Prefer tolled arcs
	}

	vector<path> paths(K);

	LOOP(k, K) {
		path p = lgraph.shortest_path(prob.commodities[k].origin, prob.commodities[k].destination);
		paths[k] = std::move(p);
	}

	return paths;
}
