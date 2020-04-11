#pragma once

#include <vector>
#include <tuple>
#include <map>
#include <set>

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
	using iipair = std::pair<int, int>;
	using toll_set = std::set<iipair>;
	using toll_list = std::vector<iipair>;

	int V;
	std::vector<light_edge> Eall;
	std::vector<std::map<int, light_edge&>> E;
	std::vector<std::map<int, light_edge&>> Er;
	std::vector<int> temp_enabled_V;

	light_graph(const problem_base::graph_type& graph);

	light_edge& edge(int src, int dst);
	light_edge& edge(const iipair& pair);
	void set_toll_arcs_enabled(bool enabled);
	void clear_toll();

	path shortest_path(int from, int to);
	double get_path_cost(const path& p);
	toll_set get_toll_set(const path& p);
	toll_list get_toll_list(const path& p);
	bool is_toll_free(const path& p);

	// Yen's algorithm
	std::vector<path> k_shortest_paths(int from, int to, int k, bool toll_free_break = false);
	void clear_temp_states();

	std::vector<path> toll_unique_paths(int from, int to, int k);
	std::vector<path> bilevel_feasible_paths_yen(int from, int to, int k);

	std::vector<path> bilevel_feasible_paths(int from, int to, int k);

	// All nodes smallest costs
	std::vector<cost_type> price_from_src(int src);
	std::vector<cost_type> price_to_dst(int dst);

	// Master routine
	bool dijkstra(int from, std::vector<cost_type>& distances, std::vector<int>& parents, int to = -1, bool reversed = false);
};
