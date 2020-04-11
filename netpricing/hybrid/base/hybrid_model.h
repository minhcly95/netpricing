#pragma once

#include "../../models/model_cplex.h"

#include <vector>

struct formulation;

struct hybrid_model : public model_with_generic_callbacks, public model_single {
	VarArray t;
	IloObjective obj;

	std::vector<formulation*> all_formulations;

	hybrid_model(IloEnv& env, const problem& prob);
	virtual ~hybrid_model();

	void formulate();
	virtual std::vector<formulation*> assign_formulations() = 0;

	// Inherited via model_with_generic_callbacks
	virtual bool solve_impl() override;
	virtual solution get_solution() override;
	virtual std::vector<std::pair<IloCplex::Callback::Function*, ContextId>> attach_callbacks() override;
};