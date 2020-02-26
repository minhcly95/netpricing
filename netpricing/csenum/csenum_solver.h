#pragma once

#include "../model.h"
#include "../graph/light_graph.h"
#include "../csenum/csenum_def.h"

struct csenum_solver : public model_single, public cplex_def
{
	light_graph primal_lgraph;
	light_graph::path primal_result;

	IloEnv env;
	IloModel dual_model;
	IloCplex dual_cplex;

	// Dual model
	VarMatrix lambda;
	VarArray t;
	IloObjective dual_obj;
	RangeMatrix dual_feas;

	// State management
	std::vector<int> primal_state_stack;
	std::vector<csenum_coor> dual_state_stack;

	csenum_solver(const IloEnv& env, const problem& prob);
	csenum_solver(const csenum_solver& other);

	// Models
	void build_dual_model();

	bool solve_primal(int k);
	double get_primal_cost();
	std::vector<int> get_path();

	bool solve_dual();
	double get_dual_cost();

	NumMatrix get_lambda();
	NumArray get_t();
	NumMatrix get_lambda(NumMatrix& lvals);
	NumArray get_t(NumArray& tvals);

	// State management
	void clear_primal_state();
	void clear_dual_state();

	void set_primal_state(std::vector<int> as);
	void set_dual_state(std::vector<csenum_coor> coors);

	void push_primal_state(int a);
	void push_dual_state(const csenum_coor& coor);

	void pop_primal_state();
	void pop_dual_state();
};
