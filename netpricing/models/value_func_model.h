#pragma once

#include "model.h"
#include <unordered_map>
#include <atomic>
#include <mutex>

struct problem;

struct value_func_model : public model_with_callbacks, public model_single {
	// Variables
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

	// Solution injection
	std::atomic<bool> sol_pending;
	std::atomic<IloNum> sol_obj;
	NumVarArray sol_vars;
	NumArray sol_vals;
	std::mutex sol_mutex;

	// Utilities
	double separate_time;
	double subprob_time;
	int separate_count;

	value_func_model(IloEnv& env, const problem& prob);

	void init_variable_name();

	void separate(const NumArray& tvals,
				  ConstraintArray& cuts, NumMatrix& xvals, NumMatrix& yvals, IloNum& obj);
	void separate_inner(const NumArray& tvals,
						ConstraintArray& cuts, NumMatrix& xvals, NumMatrix& yvals, IloNum& obj);

	IloCplex::Callback attach_callback(IloCplex& cplex);

	// Inherited via model_with_callback
	virtual solution get_solution() override;
	virtual std::string get_report() override;
	virtual std::vector<IloCplex::Callback> attach_callbacks() override;
};


