#pragma once

#include "bb_queue.h"

#include <set>

template <typename node_type, bb_opt_direction opt_dir>
struct queue_best_first : public bb_queue<node_type, opt_dir>
{
	std::multiset<node_type*, bb_better<opt_dir, node_type>> mset;

	queue_best_first() : mset() {}

	virtual int size() const override;
	virtual node_type* next() const override;
	virtual void pop() override;
	virtual void append(const std::vector<node_type*>& nodes) override;
	virtual void prune(double new_obj) override;
	virtual double get_best_bound() const override;
};