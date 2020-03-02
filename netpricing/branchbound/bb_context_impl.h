#pragma once

#include "bb_context.h"

#include <algorithm>
#include <utility>
#include <iostream>

template <typename _queue_type>
inline bb_context<_queue_type>::bb_context() : queue(), branch_cat_count{0,0,0}
{
	best_obj = std::numeric_limits<double>::infinity();
	if (opt_dir == Maximize)
		best_obj = -best_obj;
	best_node = nullptr;

	time_limit = 0;
	print_interval = 5; // seconds
	last_print_time = -std::numeric_limits<double>::infinity();

	node_count = 0;
	step_count = 0;
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

		// Delete the old node
		delete node;

		// Statistics
		step_count++;
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
		// No feasible branch, prioritize this candidate (this node could be pruned)
		if (min_impr < 0) {
			best_candidate = &candidate;
			break;
		}

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
		node_type* child_node = new node_type();
		child_node->id = node_count++;
		child_node->parent = node->id;
		child_node->lineage_node = make_shared<bb_lineage_node<candidate_type>>(node->lineage_node, *best_candidate, branch_dir);

		// Check if it is feasible
		bool is_feasible = update_bound(child_node, node, *best_candidate, branch_dir);

		// If feasible, log the improvement and add to the nodes list
		if (is_feasible) {
			// Record the improvement
			double impr = abs(child_node->get_bound() - node->get_bound());
			impr_history[make_tuple(*best_candidate, branch_dir)].push(impr);

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

template <typename _queue_type>
inline double bb_context<_queue_type>::get_impr_avg(const candidate_type& candidate, bool branch_dir)
{
	return impr_history[std::make_tuple(candidate, branch_dir)].average();
}

template<typename _queue_type>
inline void bb_context<_queue_type>::print_header() const
{
	std::cout << "   Step   Left  Depth     Curr Bnd   Best Bound     Best obj  Gap %   Time         Index Parent   Branch    Cut Infeas" << std::endl;
}

template<typename _queue_type>
inline void bb_context<_queue_type>::print_node(node_type* node, bool is_solution) const
{
	printf("%s%6d %6d  %5d   %10.2f   %10.2f   %10.2f %6.2f %6.0f        %6d %6d   %6d %6d %6d",
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
		   get_branch_category_count(2),
		   get_branch_category_count(1),
		   get_branch_category_count(0)
		   );
	std::cout << std::endl;
}
