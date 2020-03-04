#pragma once

#include "csenum_solver_dual_only.h"
#include "../graph/light_graph.h"

struct csenum_solver : public csenum_solver_dual_only
{
	std::vector<light_graph> primal_lgraphs;
	std::vector<light_graph::path> primal_results;

	csenum_solver(const IloEnv& env, const problem& prob);

	// Models
	void build_primal_model();

	virtual bool solve_primal(int k) override;
	virtual std::vector<int> get_path(int k) override;
	virtual double get_primal_cost(const std::vector<int>& path) override;

	// State management
	virtual void clear_primal_state_impl() override;
	virtual void push_primal_state_impl(const csenum_coor& coor) override;
	virtual void pop_primal_state_impl(const csenum_coor& coor) override;
};
