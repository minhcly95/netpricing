#include "processed_hmodel.h"

#include "../macros.h"
#include "formulations/null_formulation.h"

#include "formulations/standard_formulation.h"
#include "formulations/value_func_formulation.h"

using namespace std;

template <class alternative>
processed_hmodel<alternative>::processed_hmodel(IloEnv& env, const problem& prob) :
	hybrid_model(env, prob), lgraph(prob.graph)
{
}

template <class alternative>
vector<formulation*> processed_hmodel<alternative>::assign_formulations()
{
	int filtered_count = 0, processed_count = 0, normal_count = 0;

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
			all_forms[k] = new alternative(paths);
			processed_count++;
		}
		else {
			all_forms[k] = new alternative();
			normal_count++;
		}
	}

	cout << "FILTERED: " << filtered_count << endl;
	cout << "PROCESSED: " << processed_count << endl;
	cout << "NORMAL: " << normal_count << endl;

	return all_forms;
}

template <class alternative>
void processed_hmodel<alternative>::config(const model_config& conf)
{
	hybrid_model::config(conf);
	max_paths = conf.max_paths;
}

template struct processed_hmodel<standard_formulation>;
template struct processed_hmodel<value_func_formulation>;