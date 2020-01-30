#include "follower_solver.h"
#include "../macros.h"

#include <algorithm>
#include <boost/graph/dijkstra_shortest_paths.hpp>

using namespace std;
using namespace boost;

follower_solver::follower_solver(const problem& _prob) : follower_solver_base(_prob)
{
	// Copy a new cost map
	problem::edge_iterator ei, ei_end;
	for (tie(ei, ei_end) = edges(prob.graph); ei != ei_end; ++ei)
		tolled_cost_map[*ei] = prob.cost_map[*ei];
}

vector<follower_solver::path> follower_solver::solve_impl(const vector<cost_type>& tolls)
{
	// Add toll to new cost map
	LOOP(a, A1) {
		auto edge = A1_TO_EDGE(prob, a);
		tolled_cost_map[edge] = prob.cost_map[edge] + tolls[a] * 0.9999;	// Prefer tolled arcs
	}

	vector<path> paths(K);
	vector<int> parents(V);
	auto index_map = get(vertex_index, prob.graph);
	auto cost_map = make_assoc_property_map(tolled_cost_map);
	auto parent_map = make_iterator_property_map(parents.begin(), index_map);

	LOOP(k, K) {
		// Find the shortest path
		int orig = prob.commodities[k].origin;
		dijkstra_shortest_paths(prob.graph, orig, weight_map(cost_map).predecessor_map(parent_map));

		// Trace back the path
		int curr = prob.commodities[k].destination;
		while (curr != orig) {
			paths[k].push_back(curr);
			curr = parents[curr];
		}
		paths[k].push_back(orig);

		std::reverse(paths[k].begin(), paths[k].end());
	}

	return paths;
}
