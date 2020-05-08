#pragma once

#include "base/hybrid_model.h"
#include "../graph/light_graph.h"

struct standard_formulation;
struct value_func_formulation;

template <class alternative>
struct spgm_hmodel : public hybrid_model {
	light_graph lgraph;

	spgm_hmodel(IloEnv& env, const problem& prob);

	// Inherited via hybrid_model
	virtual std::vector<formulation*> assign_formulations() override;
};

using spgm_standard_hmodel = spgm_hmodel<standard_formulation>;
using spgm_value_func_hmodel = spgm_hmodel<value_func_formulation>;