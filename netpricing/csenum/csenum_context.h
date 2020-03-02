#pragma once

#include "csenum_solver.h"
#include "csenum_node.h"
#include "../branchbound/queue_hybrid.h"
#include "../branchbound/bb_context.h"

struct csenum_queue : public queue_hybrid<csenum_node, Maximize> {};

struct csenum_context : public bb_context<csenum_queue> {
	IloEnv env;
	csenum_solver solver;

	problem& prob;
	int K, V, A, A1, A2;

	csenum_context(IloEnv& env, const problem& prob);

	// Inherited via bb_context
	virtual bool update_root_bound(node_type* node) override;
	virtual bool update_bound(node_type* node, node_type* parent_node, const candidate_type& candidate, bool branch_dir) override;
	virtual double evaluate_branch(node_type* node, const candidate_type& candidate, bool branch_dir) override;

	virtual void enter_node(node_type* node) override;

	// Helper
	void update_slack_map(node_type* node);
	void update_candidate_list(node_type* node);
};
