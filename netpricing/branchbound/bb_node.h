#pragma once

#include <vector>
#include "bb_lineage_node.h"

template <typename _candidate_type>
struct bb_node {
	using candidate_type = _candidate_type;

	int id;
	int parent;
	typename bb_lineage_node<candidate_type>::ptr_type lineage_node;

	bb_node() : id(0), parent(-1), lineage_node(nullptr) {}

	virtual bb_node* clone() const = 0;

	virtual double get_bound() const = 0;
	virtual const std::vector<candidate_type>& get_candidates() const = 0;

	bool is_solution() const {
		return get_candidates().size() == 0;
	}
	int get_depth() const {
		if (lineage_node == nullptr)
			return 0;
		else
			return lineage_node->get_depth();
	}
};
