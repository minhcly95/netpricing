#pragma once

#include "../../models/model_cplex.h"

#include <vector>

struct formulation;

struct hybrid_model : public model_with_generic_callback, public model_single {
	VarArray t;
	IloObjective obj;

	std::vector<formulation*> all_formulations;

	double cb_time;
	double cb_count;

	hybrid_model(IloEnv& env, const problem& prob);
	virtual ~hybrid_model();

	void formulate();
	virtual std::vector<formulation*> assign_formulations() = 0;

	// Inherited via model_with_generic_callbacks
	virtual bool solve_impl() override;
	virtual solution get_solution() override;
	virtual std::pair<IloCplex::Callback::Function*, ContextId> attach_callback() override;
	virtual std::string get_report() override;
};