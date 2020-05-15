#include "arc_path_hmodel.h"

#include "../macros.h"
#include "formulations/null_formulation.h"

#include "formulations/standard_formulation.h"
#include "formulations/value_func_formulation.h"
#include "formulations/arc_path_standard_formulation.h"
#include "formulations/arc_path_value_func_formulation.h"

#include "preprocessors/path_preprocessor.h"

using namespace std;

template <class arc_path_form, class alternative>
arc_path_hmodel<arc_path_form, alternative>::arc_path_hmodel(IloEnv& env, const problem& prob) :
	hybrid_model(env, prob), lgraph(prob.graph)
{
}

template <class arc_path_form, class alternative>
vector<formulation*> arc_path_hmodel<arc_path_form, alternative>::assign_formulations()
{
	int filtered_count = 0, arc_path_count = 0, fallback_count = 0;

	std::vector<formulation*> all_forms(K);
	LOOP(k, K) {
		auto paths = lgraph.bilevel_feasible_paths(prob.commodities[k].origin,
												   prob.commodities[k].destination,
												   max_paths + 1);
		if (paths.size() <= 1) {
			all_forms[k] = new null_formulation();
			filtered_count++;
		}
		else if (paths.size() <= max_paths) {
			all_forms[k] = new arc_path_form(paths);
			arc_path_count++;
		}
		else {
			all_forms[k] = new alternative();
			fallback_count++;
		}
	}

	cout << "FILTERED: " << filtered_count << endl;
	cout << "ARC-PATH: " << arc_path_count << endl;
	cout << "FALLBACK: " << fallback_count << endl;

	return all_forms;
}

template <class arc_path_form, class alternative>
void arc_path_hmodel<arc_path_form, alternative>::config(const model_config& conf)
{
	hybrid_model::config(conf);
	max_paths = conf.max_paths;
}

template struct arc_path_hmodel<arc_path_standard_formulation, standard_formulation>;
template struct arc_path_hmodel<arc_path_standard_formulation, value_func_formulation>;
template struct arc_path_hmodel<arc_path_value_func_formulation, standard_formulation>;
template struct arc_path_hmodel<arc_path_value_func_formulation, value_func_formulation>;