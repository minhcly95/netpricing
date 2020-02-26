#pragma once

#include <vector>

template <typename _candidate_type>
struct bb_node {
	using candidate_type = _candidate_type;

	int id;
	int parent;
	double bound;
	std::vector<candidate_type> candidates;

	virtual bb_node* clone() const = 0;

	double get_bound() const {
		return bound;
	}
	const std::vector<candidate_type>& get_candidates() const {
		return candidates;
	}
	bool is_solution() const {
		return get_candidates().size() == 0;
	}
};
