#pragma once

#include "model_cplex.h"

struct problem;

struct slackbranch_model : public model_with_goal, public model_single {
	// Variables
	NumVarMatrix x;
	NumVarMatrix y;
	NumVarMatrix z;
	NumVarMatrix lambda;
	NumVarMatrix tx;
	IloNumVarArray t;

	// Objective and constraints
	IloObjective obj;
	RangeMatrix flow_constr;
	RangeMatrix dual_feas;
	RangeArray equal_obj;
	RangeMatrix bilinear1;
	RangeMatrix bilinear2;
	RangeMatrix bilinear3;

	// Utilities
	double goal_time;

	slackbranch_model(IloEnv& env, const problem& prob);

	virtual solution get_solution() override;
	virtual std::string get_report() override;
};


