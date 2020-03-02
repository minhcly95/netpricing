#pragma once

#include "queue_hybrid.h"

#include <algorithm>

template <typename node_type, bb_opt_direction opt_dir>
inline int queue_hybrid<node_type, opt_dir>::size() const
{
	return mset.size() + (next_node == nullptr ? 0 : 1);
}

template <typename node_type, bb_opt_direction opt_dir>
inline node_type* queue_hybrid<node_type, opt_dir>::next() const
{
	return next_node == nullptr  ? *mset.begin() : next_node;
}

template <typename node_type, bb_opt_direction opt_dir>
inline void queue_hybrid<node_type, opt_dir>::pop()
{
	if (next_node == nullptr)
		mset.erase(mset.begin());
	else
		next_node = nullptr;
}

template <typename node_type, bb_opt_direction opt_dir>
inline void queue_hybrid<node_type, opt_dir>::append(const std::vector<node_type*>& nodes)
{
	if (nodes.empty())
		return;

	// Get the best child, set it to be the immediate next node
	auto best_node_it = std::min_element(nodes.begin(), nodes.end(), bb_better<opt_dir, node_type>());
	if (next_node != nullptr)
		mset.insert(next_node);
	next_node = *best_node_it;

	// Add the rest to the multiset
	mset.insert(nodes.begin(), best_node_it);
	std::advance(best_node_it, 1);
	mset.insert(best_node_it, nodes.end());
}

template <typename node_type, bb_opt_direction opt_dir>
inline void queue_hybrid<node_type, opt_dir>::prune(double new_obj)
{
	auto remove_it = mset.lower_bound(new_obj);
	for (auto it = remove_it; it != mset.end(); it++) {
		delete (*it);
	}
	mset.erase(remove_it, mset.end());

	if (next_node && !is_better_obj<opt_dir>(next_node->get_bound(), new_obj)) {
		delete next_node;
		next_node = nullptr;
	}
}

template<typename node_type, bb_opt_direction opt_dir>
inline double queue_hybrid<node_type, opt_dir>::get_best_bound() const
{
	double mset_obj = mset.empty() ? default_obj<opt_dir>() : (*mset.begin())->get_bound();
	double next_obj = next_node == nullptr ? default_obj<opt_dir>() : next_node->get_bound();
	return is_better_obj<opt_dir>(mset_obj, next_obj) ? mset_obj : next_obj;
}
