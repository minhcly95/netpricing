#pragma once

#include "base/hybrid_model.h"

struct value_func_hmodel : public hybrid_model {

	value_func_hmodel(IloEnv& env, const problem& prob);

	// Inherited via hybrid_model
	virtual std::vector<formulation*> assign_formulations() override;
};