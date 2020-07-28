#pragma once

#include "../../model.h"

struct hybrid_model;

struct formulation : public cplex_def {
	hybrid_model* model;
	int k;

	IloEnv env;
	problem* prob;

	IloModel cplex_model;
	IloObjective obj;
	VarArray t;

	int K, V, A, A1, A2;

	void formulate(hybrid_model* model, int k);
	virtual void formulate_impl() = 0;

	virtual std::vector<IloNumVar> get_all_variables() = 0;
	virtual IloExpr get_obj_expr() = 0;

	virtual std::vector<int> get_optimal_path() { return {}; }

	// Convert a path to a solution (must implemented bc of heuristic)
	virtual std::vector<std::pair<IloNumVar, IloNum>> path_to_solution(const NumArray& tvals,
																  const std::vector<int>& path) = 0;

	// Callback to add lazy constraint
	virtual bool has_callback() { return false; }
	virtual void invoke_callback(const IloCplex::Callback::Context& context, const NumArray& tvals) {}

	// If the callback also finds the optimal path, then it can be used to generate a heuristic solution
	virtual bool has_callback_optimal_path() { return false; }
	virtual std::vector<int> get_callback_optimal_path() { return {}; }
	virtual double get_callback_obj() { return 0; }

	// If this formulation can use a path to generate a cut, this should be implemented
	virtual bool has_heuristic_cut() { return false; }
	virtual void post_heuristic_cut(const IloCplex::Callback::Context& context, const NumArray& tvals,
									const std::vector<int>& path) { }
};
