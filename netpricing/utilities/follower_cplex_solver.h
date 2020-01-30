#pragma once

#include <ilcplex/ilocplex.h>

#include "follower_solver_base.h"

struct follower_cplex_solver : public follower_solver_base, public cplex_def
{
	IloEnv env;
	IloArray<IloModel> cplex_model;
	IloArray<IloCplex> cplex;

	NumVarMatrix x;
	NumVarMatrix y;
	NumVarMatrix z;

	IloArray<IloObjective> obj;
	RangeMatrix flow_constr;

	follower_cplex_solver(IloEnv& env, const problem& prob);
	virtual ~follower_cplex_solver();

	virtual std::vector<path> solve_impl(const std::vector<cost_type>& tolls) override;
};
