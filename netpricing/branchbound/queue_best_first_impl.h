#pragma once

#include "queue_best_first.h"

#include <algorithm>

template <typename _node_type, bb_opt_direction _opt_dir>
inline int queue_best_first<_node_type, _opt_dir>::size() const
{
	return mset.size();
}

template <typename _node_type, bb_opt_direction _opt_dir>
inline _node_type* queue_best_first<_node_type, _opt_dir>::next() const
{
	return *mset.begin();
}

template <typename _node_type, bb_opt_direction _opt_dir>
inline void queue_best_first<_node_type, _opt_dir>::pop()
{
	mset.erase(mset.begin());
}

template <typename _node_type, bb_opt_direction _opt_dir>
inline void queue_best_first<_node_type, _opt_dir>::append(const std::vector<node_type*>& nodes)
{
	mset.insert(nodes.begin(), nodes.end());
}

template <typename _node_type, bb_opt_direction _opt_dir>
inline void queue_best_first<_node_type, _opt_dir>::prune(double new_obj)
{
	mset.erase(mset.lower_bound(new_obj), mset.end());
}
