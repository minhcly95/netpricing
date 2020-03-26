#pragma once

#include "model_cplex.h"
#include <unordered_map>

struct problem;

struct benders_xy_model : public model_with_callback, public model_single {
	// Subproblem
	IloModel submodel;
	IloCplex subcplex;

	// Variables
	IloNumVar v;
	NumVarMatrix x;
	NumVarMatrix y;
	NumVarMatrix lambda;
	NumVarMatrix tx;
	IloNumVarArray t;

	// Objective and constraints
	IloObjective obj;
	IloObjective subobj;
	RangeMatrix flow_constr;
	RangeMatrix dual_feas;
	RangeArray equal_obj;
	RangeMatrix bilinear1;
	RangeMatrix bilinear2;
	RangeMatrix bilinear3;

	// Valid inequalities
	RangeMatrix unique_out;
	RangeMatrix unique_in;

	// Dual values
	NumMatrix f;
	NumArray rho;
	NumMatrix alpha;
	NumMatrix beta;

	// This map is used in separate function
	std::unordered_map<IloInt, IloNum*> const_val_map;

	// Utilities
	double separate_time;
	double subprob_time;
	int separate_count;

	benders_xy_model(IloEnv& env, const problem& prob);

	void add_valid_inequalities();

	void init_subproblem();
	void update_subproblem(const NumMatrix& xvals, const NumMatrix& yvals);

	void update_const_val_map();

	void separate(const NumMatrix& xvals, const NumMatrix& yvals, RangeArray& cuts);
	void separate_inner(const NumMatrix& xvals, const NumMatrix& yvals, RangeArray& cuts);

	// Inherited via model_with_callback
	virtual solution get_solution() override;
	virtual std::string get_report() override;
	virtual IloCplex::Callback attach_callback() override;
};


