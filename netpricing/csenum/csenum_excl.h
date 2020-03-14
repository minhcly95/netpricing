#pragma once

#include "csenum_context.h"

struct csenum_excl : public model_base, public cplex_def {
	using problem_type = problem;

	csenum_context context;

	// Constructor
	csenum_excl(IloEnv& env, const problem& prob);

	// Inherited via model_base
	virtual bool solve_impl() override;
	virtual solution get_solution() override;
	virtual void config(const model_config& config) override;

	virtual double get_best_obj() override;
	virtual double get_best_bound() override;
	virtual double get_gap() override;
	virtual int get_step_count() override;
};
