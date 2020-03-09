#pragma once

#include "../problem.h"
#include "../model.h"

struct inverse_solver : public model_single, cplex_def
{
	using path = std::vector<int>;
	double time;

	IloEnv env;
	IloModel cplex_model;
	IloCplex cplex;

	IloObjective obj;
	VarMatrix lambda;
	VarArray t;
	RangeMatrix dual_feas;

	inverse_solver(IloEnv& env, const problem& prob);

	std::vector<cost_type> solve(const std::vector<path>& paths);
	virtual std::vector<cost_type> solve_impl(const std::vector<path>& paths);
};
