#pragma once

#include <chrono>
#include <map>
#include <tuple>

#include "bb_queue.h"

struct bb_improvement_entry {
	double sum;
	int count;

	bb_improvement_entry() : sum(0), count(0) {}
	double average() { return count > 0 ? sum / count : 0; }
	void push(double new_impr) {
		sum += new_impr;
		count++;
	}
};

template <typename _queue_type>
struct bb_context {
	using queue_type = _queue_type;
	using node_type = queue_type::node_type;
	using candidate_type = node_type::candidate_type;
	using improvement_key = std::tuple<candidate_type, bool>;

	static constexpr bb_opt_direction opt_dir = queue_type::opt_dir;

	queue_type queue;

	double best_obj;
	node_type* best_node;

	std::map<improvement_key, bb_improvement_entry> impr_history;

	std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
	double time_limit;

	bb_context();
	node_type* add_node(const node_type* node);

	void solve();
	void step(node_type* node);

	void add_new_solution(node_type* node);
	double get_impr_avg(const candidate_type& candidate, bool branch_dir);

	virtual bool update_bound(node_type* node) = 0;
	virtual double evaluate_branch(node_type* node, const candidate_type& candidate, bool branch_dir) = 0;
	virtual bb_node* create_child(node_type* parent, const candidate_type& candidate, bool branch_dir) = 0;

	// Optional callbacks
	virtual void enter_node(node_type* node) {}
};