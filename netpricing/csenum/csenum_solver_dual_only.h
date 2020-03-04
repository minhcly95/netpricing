#pragma once

#include "csenum_solver_base.h"

struct csenum_solver_dual_only : public csenum_solver_base
{
	IloModel dual_model;
	IloCplex dual_cplex;

	// Dual model
	VarMatrix lambda;
	VarArray t;
	IloObjective dual_obj;
	RangeMatrix dual_feas;

	csenum_solver_dual_only(const IloEnv& env, const problem& prob);

	// Models
	void build_dual_model();

	virtual bool solve_dual() override;
	virtual double get_dual_cost() override;

	virtual NumMatrix get_lambda_impl(NumMatrix& lvals) override;
	virtual NumArray get_t_impl(NumArray& tvals) override;

	// State management
	virtual void clear_dual_state_impl() override;
	virtual void push_dual_state_impl(const csenum_coor& coor) override;
	virtual void pop_dual_state_impl(const csenum_coor& coor) override;
};
