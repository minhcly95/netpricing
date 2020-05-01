#pragma once

#include "base/hybrid_model.h"
#include "../graph/light_graph.h"

struct standard_formulation;
struct value_func_formulation;

template <class alternative>
struct processed_hmodel : public hybrid_model {
	light_graph lgraph;
	int max_paths;

	processed_hmodel(IloEnv& env, const problem& prob);

	// Inherited via hybrid_model
	virtual std::vector<formulation*> assign_formulations() override;

	virtual void config(const model_config& conf) override;
};

using processed_standard_hmodel = processed_hmodel<standard_formulation>;
using processed_value_func_hmodel = processed_hmodel<value_func_formulation>;