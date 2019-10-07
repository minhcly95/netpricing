#pragma once

#include "model.h"

struct problem;

struct standard_model : public model {

	using NumVarArray = IloNumVarArray;
	using NumVarMatrix = IloArray<NumVarArray>;

	using RangeArray = IloRangeArray;
	using RangeMatrix = IloArray<RangeArray>;

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

	standard_model(IloEnv& env, const problem& prob);
};

