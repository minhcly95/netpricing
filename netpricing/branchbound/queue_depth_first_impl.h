#pragma once

#include "queue_depth_first.h"

#include <algorithm>

template <typename node_type, bb_opt_direction opt_dir>
inline int queue_depth_first<node_type, opt_dir>::size() const
{
	return stack.size();
}

template <typename node_type, bb_opt_direction opt_dir>
inline node_type* queue_depth_first<node_type, opt_dir>::next() const
{
	return stack.back();
}

template <typename node_type, bb_opt_direction opt_dir>
inline void queue_depth_first<node_type, opt_dir>::pop()
{
	stack.pop_back();
}

template <typename node_type, bb_opt_direction opt_dir>
inline void queue_depth_first<node_type, opt_dir>::append(const std::vector<node_type*>& nodes)
{
	stack.insert(stack.end(), nodes.begin(), nodes.end());
}

template <typename node_type, bb_opt_direction opt_dir>
inline void queue_depth_first<node_type, opt_dir>::prune(double new_obj)
{
	auto remove_it = std::remove_if(stack.begin(),
									stack.end(),
									[new_obj, this](node_type* node) {
										return !is_better_obj<opt_dir>(node->get_bound(), new_obj);
									});
	
	for (auto it = remove_it; it != stack.end(); it++) {
		delete (*it);
	}
	stack.erase(remove_it, stack.end());
}

template<typename node_type, bb_opt_direction opt_dir>
inline double queue_depth_first<node_type, opt_dir>::get_best_bound() const
{
	if (stack.empty())
		return default_obj<opt_dir>();
	else
		return std::min_element(stack.begin(), stack.end(), bb_better<opt_dir, node_type>());
}