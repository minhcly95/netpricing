#pragma once

#include <vector>

#include "bb_node.h"
#include "bb_opt_direction.h"

template <typename _node_type, bb_opt_direction _opt_dir>
struct bb_queue {
	using node_type = _node_type;
	static constexpr bb_opt_direction opt_dir = _opt_dir;

	virtual int size() const = 0;
	virtual node_type* next() const = 0;
	virtual void pop() = 0;
	virtual void append(const std::vector<node_type*>& nodes) = 0;
	virtual void prune(double new_obj) = 0;
	virtual double get_best_bound() const = 0;

	void push(node_type* node) {
		append(std::vector<node_type*> { node });
	}
	bool empty() const {
		return size() <= 0;
	}
};
