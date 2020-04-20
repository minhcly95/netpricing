#pragma once

#include "base/hybrid_model.h"
#include "../graph/light_graph.h"

struct standard_formulation;
struct value_func_formulation;

template <class alternative>
struct filtered_hmodel : public hybrid_model {
	light_graph lgraph;

	filtered_hmodel(IloEnv& env, const problem& prob);

	// Inherited via hybrid_model
	virtual std::vector<formulation*> assign_formulations() override;
};

using filtered_standard_hmodel = filtered_hmodel<standard_formulation>;
using filtered_value_func_hmodel = filtered_hmodel<value_func_formulation>;