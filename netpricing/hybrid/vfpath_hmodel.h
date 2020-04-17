#pragma once

#include "base/hybrid_model.h"
#include "../graph/light_graph.h"

struct standard_formulation;
struct value_func_formulation;

template <class alternative>
struct vfpath_hmodel : public hybrid_model {
	bool full_mode;
	int max_paths;

	light_graph lgraph;

	vfpath_hmodel(IloEnv& env, const problem& prob);

	// Inherited via hybrid_model
	virtual std::vector<formulation*> assign_formulations() override;

	virtual void config(const model_config& conf) override;
};

using vfpath_standard_hmodel = vfpath_hmodel<standard_formulation>;
using vfpath_value_func_hmodel = vfpath_hmodel<value_func_formulation>;