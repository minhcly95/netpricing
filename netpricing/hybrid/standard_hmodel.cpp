#include "standard_hmodel.h"

#include "../macros.h"
#include "formulations/standard_formulation.h"

using namespace std;

standard_hmodel::standard_hmodel(IloEnv& env, const problem& prob) : hybrid_model(env, prob)
{
}

vector<formulation*> standard_hmodel::assign_formulations()
{
	std::vector<formulation*> all_forms(K);
	LOOP(k, K) {
		all_forms[k] = new standard_formulation();
	}
	return all_forms;
}
