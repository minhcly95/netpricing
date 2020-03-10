#include "csenum_excl.h"
#include "../macros.h"
#include "csenum_solver_excl.h"

#include <chrono>

using namespace std;

csenum_excl::csenum_excl(IloEnv& _env, const problem& _prob) :
	context(new csenum_solver_excl(_env, _prob))
{
}

bool csenum_excl::solve_impl()
{
	return context.solve();
}

void csenum_excl::config(const model_config& config)
{
	context.time_limit = config.time_limit;
}

solution csenum_excl::get_solution()
{
	solution sol;
	if (context.best_node != nullptr) {
		// TODO: Extract paths and toll
	}
	return sol;
}

string csenum_excl::get_report()
{
	ostringstream ss;
	ss << "OBJ: " << context.get_best_obj() << endl <<
		"TIME: " << get_time() << " s" <<
		"    StrEval " << context.strong_eval_time << " s" <<
		"    Heur " << context.heur.time << " s" << endl;
	return ss.str();
}
