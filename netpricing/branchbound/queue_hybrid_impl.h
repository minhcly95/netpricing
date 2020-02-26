#pragma once

#include "queue_hybrid.h"

#include <algorithm>

template <typename _node_type, bb_opt_direction _opt_dir>
inline int queue_hybrid<_node_type, _opt_dir>::size() const
{
	return mset.size() + (next_node == nullptr ? 0 : 1);
}

template <typename _node_type, bb_opt_direction _opt_dir>
inline _node_type* queue_hybrid<_node_type, _opt_dir>::next() const
{
	return next_node == nullptr  ? *mset.begin() : next_node;
}

template <typename _node_type, bb_opt_direction _opt_dir>
inline void queue_hybrid<_node_type, _opt_dir>::pop()
{
	if (next_node == nullptr)
		mset.erase(mset.begin());
	else
		next_node = nullptr;
}

template <typename _node_type, bb_opt_direction _opt_dir>
inline void queue_hybrid<_node_type, _opt_dir>::append(const std::vector<node_type*>& nodes)
{
	if (nodes.empty())
		return;

	// Get the best child, set it to be the immediate next node
	auto best_node_it = std::min_element(nodes.begin(), nodes.end(), bb_better<opt_dir, node_type>());
	next_node = *best_node_it;

	// Add the rest to the multiset
	mset.insert(nodes.begin(), best_node_it);
	mset.insert(std::advance(best_node_it, 1), nodes.end());
}

template <typename _node_type, bb_opt_direction _opt_dir>
inline void queue_hybrid<_node_type, _opt_dir>::prune(double new_obj)
{
	mset.erase(mset.lower_bound(new_obj), mset.end());
	if (next_node && !is_better_obj<opt_dir>(next_node->get_bound(), new_obj))
		next_node = nullptr;
}
