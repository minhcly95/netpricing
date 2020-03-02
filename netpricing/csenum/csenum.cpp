#include "csenum.h"
#include "../macros.h"

#include <chrono>

using namespace std;

csenum::csenum(IloEnv& _env, const problem& _prob) :
	context(_env, _prob)
{
}

bool csenum::solve_impl()
{
	return context.solve();
}

void csenum::config(const model_config& config)
{
	context.time_limit = config.time_limit;
}

solution csenum::get_solution()
{
	solution sol;
	if (context.best_node != nullptr) {
		sol.paths = context.best_node->paths;
		// TODO: Extract toll
	}
	return sol;
}

string csenum::get_report()
{
	ostringstream ss;
	ss << "OBJ: " << context.get_best_obj() << endl <<
		"TIME: " << get_time() << " s" << endl;
	return ss.str();
}
