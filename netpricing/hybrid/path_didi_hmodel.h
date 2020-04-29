#pragma once

#include "base/hybrid_model.h"
#include "../graph/light_graph.h"

struct standard_formulation;
struct value_func_formulation;

template <class alternative>
struct path_didi_hmodel : public hybrid_model {
	bool full_mode;
	int max_paths;

	light_graph lgraph;

	path_didi_hmodel(IloEnv& env, const problem& prob);

	// Inherited via hybrid_model
	virtual std::vector<formulation*> assign_formulations() override;

	virtual void config(const model_config& conf) override;
};

using path_didi_standard_hmodel = path_didi_hmodel<standard_formulation>;
using path_didi_value_func_hmodel = path_didi_hmodel<value_func_formulation>;