#pragma once

#include "model_cplex.h"

struct problem;

struct standard_vfcut_model : public model_with_generic_callback, public model_single {
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
	double separate_time;
	double subprob_time;
	int separate_count;

	standard_vfcut_model(IloEnv& env, const problem& prob);

	void separate(const NumArray& tvals,
				  RangeArray& cuts, NumMatrix& xvals, NumMatrix& yvals, IloNum& obj);
	void separate_inner(const NumArray& tvals,
						RangeArray& cuts, NumMatrix& xvals, NumMatrix& yvals, IloNum& obj);

	// Inherited via model_with_callback
	virtual solution get_solution() override;
	virtual std::string get_report() override;
	virtual std::pair<IloCplex::Callback::Function*, ContextId> attach_callback() override;
};


