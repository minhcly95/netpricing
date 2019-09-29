#pragma once

#include "model.h"
#include <unordered_map>

struct problem;

struct benders_model_original : public model {

	using NumVarArray = IloNumVarArray;
	using NumVarMatrix = IloArray<NumVarArray>;

	using NumArray = IloNumArray;
	using NumMatrix = IloArray<NumArray>;

	using RangeArray = IloRangeArray;
	using RangeMatrix = IloArray<RangeArray>;

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

	// Dual values
	NumMatrix mu;
	NumMatrix f;
	NumArray rho;
	NumMatrix alpha;
	NumMatrix beta;

	// This map is used in separate function
	std::unordered_map<IloInt, IloNum*> const_val_map;

	benders_model_original(IloEnv& env, const problem& prob);

	void init_subproblem(IloEnv& env, const problem& prob);
	void update_subproblem(const NumMatrix& xvals);

	void update_const_val_map();

	void separate(const NumMatrix& xvals, IloExpr& cut_lhs, IloNum& cut_rhs);

	IloCplex::Callback attach_callback(IloCplex& cplex);
};


