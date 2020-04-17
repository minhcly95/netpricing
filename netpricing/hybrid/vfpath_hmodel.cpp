#include "vfpath_hmodel.h"

#include "../macros.h"
#include "formulations/path_formulation.h"

#include "formulations/standard_formulation.h"
#include "formulations/value_func_formulation.h"

using namespace std;

template <class alternative>
vfpath_hmodel<alternative>::vfpath_hmodel(IloEnv& env, const problem& prob) :
	hybrid_model(env, prob), lgraph(prob.graph)
{
}

template <class alternative>
vector<formulation*> vfpath_hmodel<alternative>::assign_formulations()
{
	int path_count = 0, alt_count = 0;

	std::vector<formulation*> all_forms(K);
	LOOP(k, K) {
		auto paths = lgraph.bilevel_feasible_paths(prob.commodities[k].origin,
												   prob.commodities[k].destination,
												   max_paths + 1);
		if (paths.size() <= max_paths) {
			all_forms[k] = new path_formulation(paths, full_mode);
			path_count++;
		}
		else {
			all_forms[k] = new alternative();
			alt_count++;
		}
	}

	cout << "PATH formulation: " << path_count << endl;
	cout << "ALTERNATIVE formulation: " << alt_count << endl;

	return all_forms;
}

template <class alternative>
void vfpath_hmodel<alternative>::config(const model_config& conf)
{
	hybrid_model::config(conf);
	full_mode = conf.full_mode;
	max_paths = conf.max_paths;

	if (max_paths <= 1)
		full_mode = true;
}

template struct vfpath_hmodel<standard_formulation>;
template struct vfpath_hmodel<value_func_formulation>;