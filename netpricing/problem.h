#pragma once

#include "problem_base.h"

struct problem : public problem_base
{
	// Main data
	graph_type graph;
	std::vector<commodity> commodities;

	// Auxiliary data
	edge_tolled_map_type is_tolled_map;
	edge_cost_map_type cost_map;

	edge_index_bimap_type tolled_index_map;
	edge_index_bimap_type tollfree_index_map;
	edge_index_bimap_type alledges_index_map;

	tollfree_graph_type tollfree_graph;
	big_m_type big_m;
	big_n_type big_n;

	// Constructor
	problem(const graph_type& graph, const std::vector<commodity>& commodities);
	problem(const graph_type& graph, std::vector<commodity>&& commodities);

	problem(const problem& problem);
	problem(problem&& problem);

	// Read and write
	static problem parse_json(const nlohmann::json& json_obj);
	static nlohmann::json get_json(const problem& prob);

	static std::vector<problem> read_from_json(std::string filename);
	static void write_to_json(std::string filename, const std::vector<problem>& problems);

	virtual nlohmann::json get_json() const override;
	void write_to_json(std::string filename) const;

	// Update auxiliary data
	void update();
	void update_indices();
	void update_big_mn();

	// Utilities
	cost_type get_obj_upper_bound() const;
};
