#pragma once

#include "bb_queue.h"

#include <set>

template <typename _node_type, bb_opt_direction _opt_dir>
struct queue_hybrid : public bb_queue<_node_type, _opt_dir>
{
	node_type* next_node;
	std::multiset<node_type*, bb_better<opt_dir, node_type>> mset;

	queue_hybrid() : next_node(nullptr), mset() {}

	virtual int size() const override;
	virtual node_type* next() const override;
	virtual void pop() override;
	virtual void append(const std::vector<node_type*>& nodes) override;
	virtual void prune(double new_obj) override;
};