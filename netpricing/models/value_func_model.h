#pragma once

#include "model.h"
#include <unordered_map>

struct problem;

struct value_func_model : public model_with_callback {

	using NumVarArray = IloNumVarArray;
	using NumVarMatrix = IloArray<NumVarArray>;

	using NumArray = IloNumArray;
	using NumMatrix = IloArray<NumArray>;

	using RangeArray = IloRangeArray;
	using RangeMatrix = IloArray<RangeArray>;

	// Variables
	IloNumVar v;
	NumVarMatrix x;
	NumVarMatrix y;
	NumVarMatrix z;
	NumVarMatrix tx;
	IloNumVarArray t;

	// Objective and constraints
	IloObjective obj;
	RangeMatrix flow_constr;
	RangeMatrix bilinear1;
	RangeMatrix bilinear2;
	RangeMatrix bilinear3;

	// Utilities
	double separate_time;
	double subprob_time;
	int separate_count;

	value_func_model(IloEnv& env, const problem& prob);

	void init_variable_name();

	void separate(const NumMatrix& zvals, const NumArray& tvals, std::vector<std::pair<IloExpr, IloNum>>& cuts);
	void separate_inner(const NumMatrix& zvals, const NumArray& tvals, std::vector<std::pair<IloExpr, IloNum>>& cuts);

	IloCplex::Callback attach_callback(IloCplex& cplex);

	// Inherited via model_with_callback
	virtual std::string get_report() override;
	virtual IloCplex::Callback attach_callback() override;
};


