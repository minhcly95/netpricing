#pragma once

#include "model_cplex.h"
#include "../utilities/follower_light_solver.h"

struct problem;

struct light_vfcut_model : public model_with_generic_callbacks, public model_single {
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

	// Light graph solver
	follower_light_solver lsolver;

	// Utilities
	double separate_time;
	double subprob_time;
	int separate_count;

	light_vfcut_model(IloEnv& env, const problem& prob);

	void init_variable_name();

	void separate(const NumArray& tvals,
				  RangeArray& cuts, NumMatrix& xvals, NumMatrix& yvals, IloNum& obj);
	void separate_inner(const NumArray& tvals,
						RangeArray& cuts, NumMatrix& xvals, NumMatrix& yvals, IloNum& obj);

	// Inherited via model_with_callback
	virtual solution get_solution() override;
	virtual std::string get_report() override;
	virtual std::vector<std::pair<IloCplex::Callback::Function*, ContextId>> attach_callbacks() override;
};


