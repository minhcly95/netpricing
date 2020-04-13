#include "value_func_hmodel.h"

#include "../macros.h"
#include "formulations/value_func_formulation.h"

using namespace std;

value_func_hmodel::value_func_hmodel(IloEnv& env, const problem& prob) : hybrid_model(env, prob)
{
}

vector<formulation*> value_func_hmodel::assign_formulations()
{
	std::vector<formulation*> all_forms(K);
	LOOP(k, K) {
		all_forms[k] = new value_func_formulation();
	}
	return all_forms;
}
