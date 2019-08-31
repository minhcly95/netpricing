#pragma once

template <class tolled_map_type>
struct edge_tollfree_predicate {
	tolled_map_type tolled_map;

	edge_tollfree_predicate() {}
	edge_tollfree_predicate(const tolled_map_type& tolled_map) : tolled_map(tolled_map) {}

	template <class edge_descriptor_type>
	bool operator()(const edge_descriptor_type& edge) const {
		return !tolled_map[edge];
	}
};
