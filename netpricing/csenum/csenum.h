#pragma once

#include "csenum_context.h"

struct csenum : public model_base, public cplex_def {
	using problem_type = problem;

	csenum_context context;

	// Constructor
	csenum(IloEnv& env, const problem& prob);

	// Inherited via model_base
	virtual bool solve_impl() override;
	virtual solution get_solution() override;
	virtual std::string get_report() override;
	virtual void config(const model_config& config) override;
};
