#pragma once

#include "../model.h"
#include "../graph/light_graph.h"
#include "../csenum/csenum_def.h"

struct csenum_solver : public model_single, public cplex_def
{
	std::vector<light_graph> primal_lgraphs;
	std::vector<light_graph::path> primal_results;

	IloEnv env;
	IloModel dual_model;
	IloCplex dual_cplex;

	// Dual model
	VarMatrix lambda;
	VarArray t;
	IloObjective dual_obj;
	RangeMatrix dual_feas;

	// State management
	std::vector<csenum_coor> primal_state_stack;
	std::vector<csenum_coor> dual_state_stack;

	csenum_solver(const IloEnv& env, const problem& prob);

	// Models
	void build_primal_model();
	void build_dual_model();

	bool solve_primal(int k);
	double get_primal_cost(int k);
	std::vector<int> get_path(int k);

	std::vector<bool> solve_primals();
	std::vector<double> get_primal_costs();
	std::vector<std::vector<int>> get_paths();

	bool solve_dual();
	double get_dual_cost();

	NumMatrix get_lambda();
	NumArray get_t();
	NumMatrix get_lambda(NumMatrix& lvals);
	NumArray get_t(NumArray& tvals);

	// State management
	void clear_primal_state();
	void clear_dual_state();

	void set_primal_state(const std::vector<csenum_coor>& coors);
	void set_dual_state(const std::vector<csenum_coor>& coors);

	void push_primal_state(const csenum_coor& coor);
	void push_dual_state(const csenum_coor& coor);

	void pop_primal_state();
	void pop_dual_state();
};
