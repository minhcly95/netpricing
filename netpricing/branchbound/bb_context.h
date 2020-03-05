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
	double print_interval;
	double last_print_time;
	int node_count;
	int step_count;
	int branch_cat_count[3];
	int strong_eval;
	double strong_eval_time;

	// Parameters
	double time_limit;
	int reliable_threshold;
	int reliable_lookahead;

	// Constructor
	bb_context();
	virtual ~bb_context();

	// Solution
	bool solve();
	void step(node_type* node);

	// Helpers
	void add_new_solution(node_type* node);
	double calculate_score(double first_impr, double second_impr);
	double get_candidate_pseudo_score(const candidate_type& candidate);
	bool is_pseudo_score_reliable(const candidate_type& candidate, int threshold);

	// Virtual
	virtual bool update_root_bound(node_type* node) = 0;
	virtual bool update_bound(node_type* node, node_type* parent_node, const candidate_type& candidate, bool branch_dir) = 0;
	virtual double evaluate_branch(node_type* node, const candidate_type& candidate, bool branch_dir) = 0;

	// Optional callbacks
	virtual void enter_node(node_type* node) {}

	// Query
	double get_current_time() const;
	double get_best_bound() const;
	double get_best_obj() const;
	double get_gap_ratio() const;
	int get_branch_category_count(int i) const;

	// Print
	void print_header() const;
	void print_node(node_type* node, bool is_solution = false) const;
};
