#pragma once

#include <vector>
#include <unordered_map>
#include <queue>

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
};

struct light_graph
{
	using path = std::vector<int>;
	using cipair = std::pair<cost_type, int>;
	using iipair = std::pair<int, int>;

	int V;
	std::vector<std::unordered_map<int, light_edge>> E;

	light_graph(const problem_base::graph_type& graph);

	path shortest_path(int from, int to);
	double get_path_cost(const path& p);
};
