#pragma once

#include <vector>
#include <tuple>
#include <map>
#include <set>

#include "../typedef.h"
#include "../problem_base.h"

struct light_edge
{
	int index;
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
	std::vector<std::map<int, int>> E;
	std::vector<std::map<int, int>> Er;
	std::vector<int> temp_enabled_V;

	light_graph(int V);
	light_graph(const problem_base::graph_type& graph);

	int num_tolled() const;

	light_edge& edge(int src, int dst);
	light_edge& edge(const iipair& pair);
	const light_edge& edge(int src, int dst) const;
	const light_edge& edge(const iipair& pair) const;
	void set_toll_arcs_enabled(bool enabled);
	void clear_toll();

	path shortest_path(int from, int to);
	cost_type get_path_cost(const path& p, bool with_toll = true) const;
	cost_type get_path_toll(const path& p) const;
	toll_set get_toll_set(const path& p) const;
	toll_list get_toll_list(const path& p) const;
	bool is_toll_free(const path& p) const;

	// Yen's algorithm
	std::vector<path> k_shortest_paths(int from, int to, int k, bool toll_free_break = false);
	void clear_temp_states();

	std::vector<path> filter_bilevel_feasible(const std::vector<path>& input) const;

	std::vector<path> toll_unique_paths(int from, int to, int k);
	std::vector<path> bilevel_feasible_paths_yen(int from, int to, int k);

	std::vector<path> bilevel_feasible_paths(int from, int to, int k);
	std::vector<path> bilevel_feasible_paths_2(int from, int to, int k, bool filter=true);

	// All nodes smallest costs
	std::vector<cost_type> price_from_src(int src) const;
	std::vector<cost_type> price_to_dst(int dst) const;

	// Master routine
	bool dijkstra(int from, std::vector<cost_type>& distances, std::vector<int>& parents, int to = -1, bool reversed = false) const;
};
