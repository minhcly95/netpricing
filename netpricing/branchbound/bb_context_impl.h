#pragma once

#include "bb_context.h"

#include <algorithm>
#include <utility>

template <typename _queue_type>
inline bb_context<_queue_type>::bb_context() : queue()
{
	best_obj = std::numeric_limits<double>::infinity();
	if (opt_dir == Maximize)
		best_obj = -best_obj;
	best_node = nullptr;

	time_limit = 0;
}

template <typename _queue_type>
inline bb_context<_queue_type>::node_type* bb_context<_queue_type>::add_node(const node_type* node)
{
	node_type* cloned = node->clone();
	queue.push(cloned);
	return cloned;
}

template <typename _queue_type>
inline void bb_context<_queue_type>::solve()
{
	using namespace std;
	start_time = chrono::high_resolution_clock::now();

	while (!queue.empty()) {
		// Fetch the node
		node_type* node = queue.next();
		queue.pop();

		// Process the node
		step(node);

		// Delete the old node
		delete node;
	}
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

	// Choose the best candidate (prioritize single branch (cut))
	double best_impr = -std::numeric_limits<double>::infinity();
	const candidate_type* best_candidate = nullptr;
	bool only_single_branch = false;

	for (const candidate_type& candidate : candidates) {
		// Evaluate the improvement (negative means infeasible branch)
		double down_branch = evaluate_branch(node, candidate, false);
		double up_branch = evaluate_branch(node, candidate, true);

		double min_impr = std::min(down_branch, up_branch);
		double max_impr = std::max(down_branch, up_branch);

		bool single_branch = false;
		// Only 1 feasible branch
		if (min_impr < 0) {
			min_impr = max_impr;
			single_branch = true;
		}
		// No feasible branch (skip this candidate)
		if (min_impr < 0)
			continue;

		// If there is a single-branch candidate and this one is not, skip
		if (only_single_branch && !single_branch)
			continue;

		// If this one is the first single-branch candidate, assign this to be the best
		if (!only_single_branch && single_branch) {
			best_impr = min_impr;
			best_candidate = &candidate;
			only_single_branch = true;
		}

		// Otherwise, normal comparison
		if (min_impr > best_impr) {
			best_impr = min_impr;
			best_candidate = &candidate;
		}
	}

	// If there is no feasible candidate (infeasible node), prune this node
	if (best_candidate == nullptr) {
		return;
	}

	vector<node_type*> children;

	for (int i = 0; i < 2; i++) {
		bool branch_dir = i;

		// Create the child node
		node_type* child_node = create_child(node, best_candidate, branch_dir);

		// Check if it is feasible
		bool is_feasible = update_bound(child_node);

		// If feasible, log the improvement and add to the nodes list
		if (is_feasible) {
			// Prune node worse than obj
			if (!is_better_obj<opt_dir>(child_node->get_bound(), best_obj))
				continue;

			// Record the improvement
			double impr = abs(child_node->get_bound() - node->get_bound());
			impr_history[make_tuple(best_candidate, branch_dir].push(impr);

			// If is a solution, test for new solution
			if (child_node->is_solution())
				add_new_solution(child_node);
			else
				children.push_back(child_node);
		}
	}

	// Add the new nodes to the queue
	queue.append(children);
}

template <typename _queue_type>
inline void bb_context<_queue_type>::add_new_solution(node_type* node)
{
	if (is_better_obj<opt_dir>(node->get_bound(), best_obj)) {
		// Save the new node
		best_obj = node->get_bound();
		best_node = node->clone();

		// Prune the tree
		queue.prune(best_obj);

		// TODO: Optional print
	}
}

template <typename _queue_type>
inline double bb_context<_queue_type>::get_impr_avg(const candidate_type& candidate, bool branch_dir)
{
	return impr_history[std::make_tuple(candidate, branch_dir)].average();
}
