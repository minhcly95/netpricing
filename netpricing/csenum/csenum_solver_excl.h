#pragma once

#include "csenum_solver_dual_only.h"
#include "../graph/light_graph.h"

struct csenum_solver_excl : public csenum_solver_dual_only
{
	std::vector<IloModel> primal_models;
	std::vector<IloCplex> primal_cplex;

	// Dual model
	VarMatrix z;
	std::vector<IloObjective> primal_objs;
	RangeMatrix flow_constr;
	RangeMatrix unique_out;
	RangeMatrix unique_in;

	csenum_solver_excl(const IloEnv& env, const problem& prob);

	// Models
	void build_primal_model();

	virtual bool solve_primal(int k) override;
	virtual std::vector<int> get_primal_arcs(int k) override;
	virtual double get_primal_cost(int k) override;

	// State management
	virtual void clear_primal_state_impl() override;
	virtual void clear_dual_state_impl() override;

	virtual void push_primal_state_impl(const csenum_coor& coor) override;
	virtual void push_dual_state_impl(const csenum_coor& coor) override;

	virtual void pop_primal_state_impl(const csenum_coor& coor) override;
	virtual void pop_dual_state_impl(const csenum_coor& coor) override;
};
