#pragma once

#include "bb_queue.h"

template <typename _node_type, bb_opt_direction _opt_dir>
struct queue_depth_first : public bb_queue<_node_type, _opt_dir>
{
	std::vector<node_type*> stack;

	queue_depth_first() : stack() {}

	virtual int size() const override;
	virtual node_type* next() const override;
	virtual void pop() override;
	virtual void append(const std::vector<node_type*>& nodes) override;
	virtual void prune(double new_obj) override;
};