#pragma once

#include "model_cplex.h"

struct problem;

struct standard_model_multi : public model_cplex, public model_multi {
	// Variables
	NumVarMatrix x;
	NumVarMatrix y;
	NumVarMatrix z;
	NumVarMatrix lambda;
	NumVarMatrix tx;
	NumVarArray t;

	// Objective and constraints
	IloObjective obj;
	RangeMatrix flow_constr;
	RangeMatrix dual_feas;
	RangeArray equal_obj;
	RangeMatrix bilinear1;
	RangeMatrix bilinear2;
	RangeMatrix bilinear3;

	standard_model_multi(IloEnv& env, const problem_multi& prob);

	virtual solution get_solution() override;
	virtual std::string get_report() override;
};


