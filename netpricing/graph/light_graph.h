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
	cost_type toll;
	bool enabled;
};

struct light_graph
{
	using path = std::vector<int>;
	using cipair = std::pair<cost_type, int>;

	int V;
	std::vector<std::unordered_map<int,light_edge>> E;

	light_graph(const problem_base::graph_type& graph);

	path shortest_path(int from, int to);
};
