#pragma once

#include "../model.h"
#include "../csenum/csenum_def.h"

struct csenum_solver_base : public model_single, public cplex_def
{
	IloEnv env;

	// State management
	std::vector<csenum_coor> primal_state_stack;
	std::vector<csenum_coor> dual_state_stack;

	csenum_solver_base(const IloEnv& env, const problem& prob);

	virtual bool solve_primal(int k) = 0;
	virtual std::vector<int> get_primal_arcs(int k) = 0;
	virtual double get_primal_cost(int k) = 0;

	std::vector<bool> solve_primals();
	std::vector<std::vector<int>> get_all_primal_arcs();
	std::vector<double> get_primal_costs();

	virtual bool solve_dual() = 0;
	virtual double get_dual_cost() = 0;

	virtual NumMatrix get_lambda_impl(NumMatrix& lvals) = 0;
	virtual NumArray get_t_impl(NumArray& tvals) = 0;

	NumMatrix get_lambda(NumMatrix& lvals);
	NumArray get_t(NumArray& tvals);
	NumMatrix get_lambda();
	NumArray get_t();

	// State management
	void clear_primal_state();
	void clear_dual_state();

	void set_primal_state(const std::vector<csenum_coor>& coors);
	void set_dual_state(const std::vector<csenum_coor>& coors);

	void push_primal_state(const csenum_coor& coor);
	void push_dual_state(const csenum_coor& coor);

	void pop_primal_state();
	void pop_dual_state();

	// Implemented in child classes
	virtual void clear_primal_state_impl() = 0;
	virtual void clear_dual_state_impl() = 0;

	virtual void push_primal_state_impl(const csenum_coor& coor) = 0;
	virtual void push_dual_state_impl(const csenum_coor& coor) = 0;

	virtual void pop_primal_state_impl(const csenum_coor& coor) = 0;
	virtual void pop_dual_state_impl(const csenum_coor& coor) = 0;
};
