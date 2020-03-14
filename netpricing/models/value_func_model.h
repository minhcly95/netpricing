#pragma once

#include "model_cplex.h"
#include "../utilities/vfcut_builder.h"
#include <unordered_map>

struct problem;

struct value_func_model : public model_with_generic_callbacks, public model_single {
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

	// Cut builder
	vfcut_builder builder;

	// Parameters
	int heur_freq;
	int pre_cut;

	// Utilities
	double presolve_time;
	int presolve_cut_count;

	value_func_model(IloEnv& env, const problem& prob);

	void init_variable_name();

	// Inherited via model_with_callback
	virtual solution get_solution() override;
	virtual std::string get_report() override;
	virtual std::vector<std::pair<IloCplex::Callback::Function*, ContextId>> attach_callbacks() override;

	virtual void presolve() override;
	virtual void config(const model_config& conf) override;
};


