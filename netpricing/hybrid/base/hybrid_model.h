#pragma once

#include "../../models/model_cplex.h"
#include "../../utilities/follower_light_solver.h"

#include <vector>

struct formulation;

struct hybrid_model : public model_with_generic_callback, public model_single {
	VarArray t;
	IloObjective obj;

	std::vector<formulation*> all_formulations;

	// Callback variables
	double cb_time;
	int cb_count;

	// Heuristic variables
	double heur_time;
	int heur_count;
	int heur_freq;
	follower_light_solver heur_solver;

	hybrid_model(IloEnv& env, const problem& prob);
	virtual ~hybrid_model();

	void formulate();
	virtual std::vector<formulation*> assign_formulations() = 0;

	// Inherited via model_with_generic_callbacks
	virtual bool solve_impl() override;
	virtual solution get_solution() override;
	virtual std::pair<IloCplex::Callback::Function*, ContextId> attach_callback() override;

	virtual std::string get_report() override;
	virtual void config(const model_config& conf) override;
};