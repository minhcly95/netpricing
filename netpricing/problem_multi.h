#pragma once

#include "problem_base.h"
#include "problem.h"

struct problem_multi : public problem_base
{
	// Main data
	std::vector<graph_type> graphs;
	std::vector<commodity> commodities;

	// Auxiliary data
	std::vector<edge_tolled_map_type> is_tolled_maps;
	std::vector<edge_cost_map_type> cost_maps;

	using src_dst_index_bimap_type = boost::bimap<int, std::pair<int, int>>;

	src_dst_index_bimap_type tolled_index_common_map;
	std::vector<edge_index_bimap_type> tolled_index_maps;
	std::vector<edge_index_bimap_type> tollfree_index_maps;
	std::vector<edge_index_bimap_type> alledges_index_maps;
	int max_edge_index;

	std::vector<tollfree_graph_type> tollfree_graphs;
	big_m_type big_m;
	big_n_type big_n;

	// Constructor
	problem_multi(const std::vector<graph_type>& graph, const std::vector<commodity>& commodities);
	problem_multi(std::vector<graph_type>&& graph, std::vector<commodity>&& commodities);

	problem_multi(const problem_multi& problem);
	problem_multi(problem_multi&& problem);

	// Constructor from normal problem
	problem_multi(const problem& problem);

	// Read and write
	static problem_multi parse_json(const nlohmann::json& json_obj);
	static nlohmann::json get_json(const problem_multi& prob);

	virtual nlohmann::json get_json() const override;
	void write_to_json(std::string filename) const;

	// Update auxiliary data
	void update();
	void update_graph_data();
	void update_indices();
	void update_big_mn();

	// Utilities
	cost_type get_obj_upper_bound() const;
};
