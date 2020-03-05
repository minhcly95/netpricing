#pragma once

#include <chrono>
#include <map>
#include <tuple>
#include <stdexcept>

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
	using node_type = typename queue_type::node_type;
	using candidate_type = typename node_type::candidate_type;
	using improvement_key = std::tuple<candidate_type, bool>;

	static constexpr bb_opt_direction opt_dir = queue_type::opt_dir;

	// Queue
	queue_type queue;

	// Best node
	double best_obj;
	node_type* best_node;

	// Improvement history
	std::map<improvement_key, bb_improvement_entry> impr_history;

	// Statistics
	std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
	double time_limit;
	double print_interval;
	double last_print_time;
	int node_count;
	int step_count;
	int branch_cat_count[3];

	// Constructor
	bb_context();
	virtual ~bb_context();

	// Solution
	bool solve();
	void step(node_type* node);

	// Helpers
	void add_new_solution(node_type* node);
	double get_impr_avg(const candidate_type& candidate, bool branch_dir);

	// Virtual
	virtual bool update_root_bound(node_type* node) = 0;
	virtual bool update_bound(node_type* node, node_type* parent_node, const candidate_type& candidate, bool branch_dir) = 0;
	virtual double evaluate_branch(node_type* node, const candidate_type& candidate, bool branch_dir) = 0;

	// Optional callbacks
	virtual void enter_node(node_type* node) {}

	// Query
	double get_current_time() const {
		return std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start_time).count();
	}
	double get_best_bound() const {
		double best_bound = queue.get_best_bound();
		return is_better_obj<opt_dir>(best_bound, best_obj) ? best_bound : best_obj;
	}
	double get_best_obj() const {
		return best_obj;
	}
	double get_gap_ratio() const {
		double bound = get_best_bound();
		double obj = get_best_obj();
		return std::abs(bound - obj) / std::min(bound, obj);
	}
	int get_branch_category_count(int i) const {
		if (i >= 0 && i <= 2)
			return branch_cat_count[i];
		throw std::invalid_argument("category must be between 0 and 2");
	}

	// Print
	void print_header() const;
	void print_node(node_type* node, bool is_solution = false) const;
};
