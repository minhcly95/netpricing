#pragma once

#include "base/hybrid_model.h"
#include "../graph/light_graph.h"

struct arc_path_standard_formulation;
struct arc_path_value_func_formulation;
struct standard_formulation;
struct value_func_formulation;

template <class arc_path_form, class alternative>
struct arc_path_hmodel : public hybrid_model {
	light_graph lgraph;
	int max_paths;

	arc_path_hmodel(IloEnv& env, const problem& prob);

	// Inherited via hybrid_model
	virtual std::vector<formulation*> assign_formulations() override;

	virtual void config(const model_config& conf) override;
};

using apstandard_standard_hmodel = arc_path_hmodel<arc_path_standard_formulation, standard_formulation>;
using apstandard_valuefunc_hmodel = arc_path_hmodel<arc_path_standard_formulation, value_func_formulation>;
using apvaluefunc_standard_hmodel = arc_path_hmodel<arc_path_value_func_formulation, standard_formulation>;
using apvaluefunc_valuefunc_hmodel = arc_path_hmodel<arc_path_value_func_formulation, value_func_formulation>;