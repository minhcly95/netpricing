#pragma once

#include "model.h"
#include <unordered_map>

struct problem;

struct benders_model_reduced2 : public model {

	using NumVarArray = IloNumVarArray;
	using NumVarMatrix = IloArray<NumVarArray>;

	using NumArray = IloNumArray;
	using NumMatrix = IloArray<NumArray>;

	using RangeArray = IloRangeArray;
	using RangeMatrix = IloArray<RangeArray>;

	// Subproblems
	IloArray<IloModel> submodel1;
	IloArray<IloCplex> subcplex1;

	IloModel submodel3;
	IloCplex subcplex3;

	// Variables
	IloNumVar v;
	NumVarMatrix x;
	NumVarMatrix y;
	NumVarMatrix lambda;
	NumVarMatrix tx;
	IloNumVarArray t;

	// Objective and constraints
	IloObjective obj;
	IloArray<IloObjective> subobj1;
	IloObjective subobj3;

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

	// Utilities
	double separate_time;
	double subprob1_time;
	double subprob2_time;
	double subprob3_time;
	int separate_count;
	int flow_cut_count;
	int path_cut_count;
	int toll_cut_count;
	int opt_cut_count;

	benders_model_reduced2(IloEnv& env, const problem& prob);

	void init_subproblem(IloEnv& env, const problem& prob);
	void update_subproblem1(const NumMatrix& xvals);
	void update_subproblem3(const NumMatrix& xvals, const NumMatrix& yvals);

	void update_const_val_map();

	void separate(const NumMatrix& xvals, IloExpr& cut_lhs, IloNum& cut_rhs);
	void separate_inner(const NumMatrix& xvals, IloExpr& cut_lhs, IloNum& cut_rhs);
	bool separate_step1(const NumMatrix& xvals, IloExpr& cut_lhs, IloNum& cut_rhs);
	bool separate_step2(const NumMatrix& xvals, const NumMatrix& yvals, IloExpr& cut_lhs, IloNum& cut_rhs);

	IloCplex::Callback attach_callback(IloCplex& cplex);
};


