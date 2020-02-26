#pragma once

#include "queue_depth_first.h"

#include <algorithm>

template <typename _node_type, bb_opt_direction _opt_dir>
inline int queue_depth_first<_node_type, _opt_dir>::size() const
{
	return stack.size();
}

template <typename _node_type, bb_opt_direction _opt_dir>
inline _node_type* queue_depth_first<_node_type, _opt_dir>::next() const
{
	return stack.back();
}

template <typename _node_type, bb_opt_direction _opt_dir>
inline void queue_depth_first<_node_type, _opt_dir>::pop()
{
	stack.pop_back();
}

template <typename _node_type, bb_opt_direction _opt_dir>
inline void queue_depth_first<_node_type, _opt_dir>::append(const std::vector<node_type*>& nodes)
{
	stack.insert(stack.end(), nodes.begin(), nodes.end());
}

template <typename _node_type, bb_opt_direction _opt_dir>
inline void queue_depth_first<_node_type, _opt_dir>::prune(double new_obj)
{
	stack.erase(std::remove_if(stack.begin(),
							   stack.end(),
							   [new_obj, this](node_type* node) {
								   return !is_better_obj<opt_dir>(node->get_bound(), new_obj);
							   }),
				stack.end());
}
