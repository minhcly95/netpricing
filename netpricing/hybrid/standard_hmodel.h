#pragma once

#include "base/hybrid_model.h"

struct standard_hmodel : public hybrid_model {

	standard_hmodel(IloEnv& env, const problem& prob);

	// Inherited via hybrid_model
	virtual std::vector<formulation*> assign_formulations() override;
};