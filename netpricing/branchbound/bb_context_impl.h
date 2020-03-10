#pragma once

#include "bb_context.h"

#include <algorithm>
#include <utility>
#include <iostream>
#include <numeric>

template <typename _queue_type>
inline bb_context<_queue_type>::bb_context() : queue(), branch_cat_count{0,0,0}
{
	best_obj = std::numeric_limits<double>::infinity();
	if (opt_dir == Maximize)
		best_obj = -best_obj;
	best_node = nullptr;

	print_interval = 5; // seconds
	last_print_time = -std::numeric_limits<double>::infinity();

	node_count = 0;
	step_count = 0;
	strong_eval = 0;
	strong_eval_time = 0;

	time_limit = 0;
	reliable_threshold = 8;
	reliable_lookahead = 4;
	heuristic_freq = 100;
}

template<typename _queue_type>
inline bb_context<_queue_type>::~bb_context()
{
	if (best_node != nullptr)
		delete best_node;
}

template <typename _queue_type>
inline bool bb_context<_queue_type>::solve()
{
	using namespace std;
	start_time = chrono::high_resolution_clock::now();

	print_header();

	// Make root node (default constructor)
	node_type* root_node = new node_type();
	node_count = 1;
	
	// Check if it is feasible
	bool is_feasible = update_root_bound(root_node);

	// If feasible, add to the queue
	if (is_feasible) {
		// If is a solution, set as new solution
		if (root_node->is_solution())
			add_new_solution(root_node);
		else
			queue.push(root_node);
	}

	// Check the queue
	while (!queue.empty()) {
		// Fetch the node
		node_type* node = queue.next();
		queue.pop();

		// Print (time-based)
		if (get_current_time() >= last_print_time + print_interval) {
			print_node(node);
			last_print_time = get_current_time();
		}

		// Process the node
		step(node);

		// Heuristic
		if (heuristic_freq > 0 && step_count % heuristic_freq == 0) {
			run_heuristic(node);
		}

		// Delete the old node
		delete node;

		// Statistics
		step_count++;

		// Time limit
		if (time_limit > 0 && get_current_time() >= time_limit)
			break;
	}

	return best_node != nullptr;
}

template <typename _queue_type>
inline void bb_context<_queue_type>::step(node_type* node)
{
	using namespace std;

	// Enter callback
	enter_node(node);

	// Get candidates
	const vector<candidate_type>& candidates = node->get_candidates();

	// Feasible solution
	if (candidates.size() == 0) {
		add_new_solution(node);
		return;
	}

	// Get the scores of candidates
	vector<double> scores(candidates.size());
	std::transform(candidates.begin(), candidates.end(), scores.begin(),
				   [&](const candidate_type& c) {
					   return get_candidate_pseudo_score(c);
				   });

	// Sort the scores (decreasing order)
	vector<int> idx(candidates.size());
	std::iota(idx.begin(), idx.end(), 0);
	std::stable_sort(idx.begin(), idx.end(),
					 [&scores](double i, double j) {
						 return scores[i] > scores[j];
					 });

	// Update the pseudo cost
	double best_score = scores[idx[0]];
	const candidate_type* best_candidate = &candidates[idx[0]];
	int lookahead_counter = 0;
	bool updated = false;		// If this is true, we don't update the pseudo cost in the branch stage to avoid repetition

	// Loop from the largest pseudo cost
	for (int i : idx) {
		const candidate_type& candidate = candidates[i];
		if (is_pseudo_score_reliable(candidate, reliable_threshold))
			continue;

		// Evaluate the improvement (negative means infeasible branch)
		auto substart = std::chrono::high_resolution_clock::now();

		double down_impr = evaluate_branch(node, candidate, false);
		double up_impr = evaluate_branch(node, candidate, true);

		auto subend = std::chrono::high_resolution_clock::now();
		strong_eval_time += std::chrono::duration<double>(subend - substart).count();
		strong_eval++;

		// Update the pseudocost
		if (down_impr >= 0)
			impr_history[make_tuple(candidate, false)].push(down_impr);
		if (up_impr >= 0)
			impr_history[make_tuple(candidate, true)].push(up_impr);

		// Test for special cases (less than 2 feasible branches)
		if (down_impr < 0 || up_impr < 0) {
			best_candidate = &candidate;
			updated = true;
			break;
		}

		// Update the best score
		double score = calculate_score(down_impr, up_impr);
		if (score > best_score) {
			best_score = score;
			best_candidate = &candidate;
			lookahead_counter = 0;
			updated = true;
		}
		else {
			lookahead_counter++;
			if (lookahead_counter >= reliable_lookahead)
				break;
		}
	}

	// Branch phase
	vector<node_type*> children;
	for (int i = 0; i < 2; i++) {
		bool branch_dir = i;

		// Create the child node
		node_type* child_node = new node_type();
		child_node->id = node_count++;
		child_node->parent = node->id;
		child_node->lineage_node = make_shared<bb_lineage_node<candidate_type>>(node->lineage_node, *best_candidate, branch_dir);

		// Check if it is feasible
		bool is_feasible = update_bound(child_node, node, *best_candidate, branch_dir);

		// If feasible, log the improvement and add to the nodes list
		if (is_feasible) {
			// Record the improvement
			if (!updated) {
				double impr = abs(child_node->get_bound() - node->get_bound());
				impr_history[make_tuple(*best_candidate, branch_dir)].push(impr);
			}

			// Prune node worse than obj
			if (!is_better_obj<opt_dir>(child_node->get_bound(), best_obj)) {
				delete child_node;
				continue;
			}

			// If is a solution, test for new solution
			if (child_node->is_solution()) {
				add_new_solution(child_node);
				delete child_node;
			}
			else
				children.push_back(child_node);
		}
		else {
			delete child_node;
		}
	}

	// Add the new nodes to the queue
	queue.append(children);

	// Statistics
	branch_cat_count[children.size()]++;
}

template <typename _queue_type>
inline void bb_context<_queue_type>::add_new_solution(node_type* node)
{
	if (is_better_obj<opt_dir>(node->get_bound(), best_obj)) {
		// Save the new node
		best_obj = node->get_bound();

		if (best_node != nullptr)
			delete best_node;

		best_node = node->clone();

		// Prune the tree
		queue.prune(best_obj);

		print_node(node, true);
	}
}

template<typename _queue_type>
inline double bb_context<_queue_type>::calculate_score(double first_impr, double second_impr)
{
	auto minmax_impr = std::minmax(first_impr, second_impr);
	return (minmax_impr.first * 5 + minmax_impr.second) / 6;
}

template <typename _queue_type>
inline double bb_context<_queue_type>::get_candidate_pseudo_score(const candidate_type& candidate)
{
	double down_cost = impr_history[std::make_tuple(candidate, false)].average();
	double up_cost = impr_history[std::make_tuple(candidate, true)].average();
	return calculate_score(down_cost, up_cost);
}

template<typename _queue_type>
inline bool bb_context<_queue_type>::is_pseudo_score_reliable(const candidate_type& candidate, int threshold)
{
	double down_count = impr_history[std::make_tuple(candidate, false)].count;
	double up_count = impr_history[std::make_tuple(candidate, true)].count;
	return down_count >= threshold && up_count >= threshold;
}

// Query
template<typename _queue_type>
inline double bb_context<_queue_type>::get_current_time() const {
	return std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start_time).count();
}

template<typename _queue_type>
inline double bb_context<_queue_type>::get_best_bound() const {
	double best_bound = queue.get_best_bound();
	return is_better_obj<opt_dir>(best_bound, best_obj) ? best_bound : best_obj;
}

template<typename _queue_type>
inline double bb_context<_queue_type>::get_best_obj() const {
	return best_obj;
}

template<typename _queue_type>
inline double bb_context<_queue_type>::get_gap_ratio() const {
	double bound = get_best_bound();
	double obj = get_best_obj();
	return std::abs(bound - obj) / std::min(bound, obj);
}

template<typename _queue_type>
inline int bb_context<_queue_type>::get_branch_category_count(int i) const {
	if (i >= 0 && i <= 2)
		return branch_cat_count[i];
	throw std::invalid_argument("category must be between 0 and 2");
}

template<typename _queue_type>
inline void bb_context<_queue_type>::print_header() const
{
	std::cout << "   Step   Left  Depth     Curr Bnd   Best Bound     Best obj  Gap %   Time         Index Parent  StrEval (Time)" << std::endl;
}

template<typename _queue_type>
inline void bb_context<_queue_type>::print_node(node_type* node, bool is_solution) const
{
	printf("%s%6d %6d  %5d   %10.2f   %10.2f   %10.2f %6.2f %6.0f        %6d %6d  %6d (%5.1f)",
		   is_solution ? "*" : " ",
		   step_count,
		   queue.size(),
		   node->get_depth(),
		   node->get_bound(),
		   get_best_bound(),
		   get_best_obj(),
		   get_gap_ratio() * 100,
		   get_current_time(),
		   node->id,
		   node->parent,
		   strong_eval,
		   strong_eval_time
		   );
	std::cout << std::endl;
}
