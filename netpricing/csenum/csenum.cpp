#include "csenum.h"
#include "../macros.h"
#include "csenum_solver.h"

#include <chrono>

using namespace std;

csenum::csenum(IloEnv& _env, const problem& _prob) :
	context(new csenum_solver(_env, _prob))
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

double csenum::get_best_obj()
{
	return context.get_best_obj();
}

double csenum::get_best_bound()
{
	return context.get_best_bound();
}

double csenum::get_gap()
{
	return context.get_gap_ratio();
}

int csenum::get_step_count()
{
	return context.step_count;
}

solution csenum::get_solution()
{
	solution sol;
	if (context.best_node != nullptr) {
		// TODO: Extract paths and toll
	}
	return sol;
}
