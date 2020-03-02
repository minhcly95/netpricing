#pragma once

#include <vector>
#include <tuple>
#include <memory>
#include <algorithm>

template <typename candidate_type>
struct bb_lineage_node
{
	using ptr_type = std::shared_ptr<bb_lineage_node<candidate_type>>;
	using data_type = std::tuple<candidate_type, bool>;

	ptr_type parent;
	data_type data;

	bb_lineage_node(ptr_type& _parent, const candidate_type& candidate, bool branch_dir);

	std::vector<data_type> get_full_lineage() const;
	int get_depth() const;
};

template<typename candidate_type>
inline bb_lineage_node<candidate_type>::bb_lineage_node(ptr_type& _parent, const candidate_type& candidate, bool branch_dir) :
	parent(_parent), data(std::make_tuple(candidate, branch_dir))
{
}

template<typename candidate_type>
inline std::vector<typename bb_lineage_node<candidate_type>::data_type> bb_lineage_node<candidate_type>::get_full_lineage() const
{
	std::vector<data_type> lineage{ data };
	ptr_type current = parent;

	// Add the branch information of the ancestors
	while (current != nullptr) {
		lineage.push_back(current->data);
		current = current->parent;
	}

	// Reverse the list
	std::reverse(lineage.begin(), lineage.end());

	return lineage;
}

template<typename candidate_type>
inline int bb_lineage_node<candidate_type>::get_depth() const
{
	int depth = 1;
	ptr_type current = parent;

	while (current != nullptr) {
		depth++;
		current = current->parent;
	}

	return depth;
}