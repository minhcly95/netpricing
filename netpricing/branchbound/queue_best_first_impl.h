#pragma once

#include "queue_best_first.h"

#include <algorithm>

template <typename node_type, bb_opt_direction opt_dir>
inline int queue_best_first<node_type, opt_dir>::size() const
{
	return mset.size();
}

template <typename node_type, bb_opt_direction opt_dir>
inline node_type* queue_best_first<node_type, opt_dir>::next() const
{
	return *mset.begin();
}

template <typename node_type, bb_opt_direction opt_dir>
inline void queue_best_first<node_type, opt_dir>::pop()
{
	mset.erase(mset.begin());
}

template <typename node_type, bb_opt_direction opt_dir>
inline void queue_best_first<node_type, opt_dir>::append(const std::vector<node_type*>& nodes)
{
	mset.insert(nodes.begin(), nodes.end());
}

template <typename node_type, bb_opt_direction opt_dir>
inline void queue_best_first<node_type, opt_dir>::prune(double new_obj)
{
	auto remove_it = mset.lower_bound(new_obj);
	for (auto it = remove_it; it != mset.end(); it++) {
		delete (*it);
	}
	mset.erase(remove_it, mset.end());
}

template<typename node_type, bb_opt_direction opt_dir>
inline double queue_best_first<node_type, opt_dir>::get_best_bound() const
{
	if (mset.empty())
		return default_obj<opt_dir>();
	else
		return next()->get_bound();
}
