#pragma once

#include "model_cplex.h"

struct problem;

struct compslack_model : public model_cplex, public model_single {
	// Variables
	VarMatrix x;
	VarMatrix y;
	VarMatrix z;
	VarMatrix lambda;
	VarArray t;

	// Objective and constraints
	IloObjective obj;
	RangeMatrix flow_constr;
	RangeMatrix dual_feas;
	RangeMatrix comp_slack;

	compslack_model(IloEnv& env, const problem& prob);

	virtual solution get_solution() override;
};


