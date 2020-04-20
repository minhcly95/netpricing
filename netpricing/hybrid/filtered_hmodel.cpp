#include "filtered_hmodel.h"

#include "../macros.h"
#include "formulations/null_formulation.h"

#include "formulations/standard_formulation.h"
#include "formulations/value_func_formulation.h"

using namespace std;

template <class alternative>
filtered_hmodel<alternative>::filtered_hmodel(IloEnv& env, const problem& prob) :
	hybrid_model(env, prob), lgraph(prob.graph)
{
}

template <class alternative>
vector<formulation*> filtered_hmodel<alternative>::assign_formulations()
{
	int filtered_count = 0;

	std::vector<formulation*> all_forms(K);
	LOOP(k, K) {
		auto paths = lgraph.bilevel_feasible_paths(prob.commodities[k].origin,
												   prob.commodities[k].destination,
												   2);
		if (paths.size() <= 1) {
			all_forms[k] = new null_formulation();
			filtered_count++;
		}
		else {
			all_forms[k] = new alternative();
		}
	}

	cout << "Eliminated: " << filtered_count << endl;

	return all_forms;
}

template struct filtered_hmodel<standard_formulation>;
template struct filtered_hmodel<value_func_formulation>;