#pragma once

#include "../base/formulation.h"

struct light_graph;

struct path_based_formulation : public formulation {
	using path = std::vector<int>;
	using toll_set = std::set<int>;		// Set of indices of toll arcs belonging to the path

	// Path attributes
	int P;
	std::vector<path> paths;
	std::vector<cost_type> null_costs;
	std::vector<toll_set> toll_sets;

	path_based_formulation(const std::vector<path>& paths);
	virtual ~path_based_formulation();

	void prepare();
};