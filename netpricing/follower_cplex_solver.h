#pragma once

#include <ilcplex/ilocplex.h>

#include "problem.h"
#include "model.h"

struct follower_cplex_solver : public model_single, public cplex_def
{
	using path = std::vector<int>;

	IloEnv env;
	IloArray<IloModel> cplex_model;
	IloArray<IloCplex> cplex;

	NumVarMatrix x;
	NumVarMatrix y;
	NumVarMatrix z;

	IloArray<IloObjective> obj;
	RangeMatrix flow_constr;

	double time;

	follower_cplex_solver(IloEnv& env, const problem& prob);
	virtual ~follower_cplex_solver();

	std::vector<path> solve(const std::vector<cost_type>& tolls);
};
