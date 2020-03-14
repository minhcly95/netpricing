#pragma once

#include <vector>
#include <unordered_map>

#include "../typedef.h"
#include "../problem_base.h"

struct light_edge
{
	int src;
	int dst;
	cost_type cost;
	bool is_tolled;
	cost_type toll;
	bool enabled;
	bool temp_enabled;
};

struct light_graph
{
	using path = std::vector<int>;

	int V;
	std::vector<std::unordered_map<int, light_edge>> E;
	std::vector<int> temp_enabled_V;

	light_graph(const problem_base::graph_type& graph);

	path shortest_path(int from, int to);
	double get_path_cost(const path& p);

	// Yen's algorithm
	std::vector<path> k_shortest_paths(int from, int to, int k, bool toll_free_break = false);
	void clear_temp_states();

	std::vector<path> toll_unique_paths(int from, int to, int k);
};
