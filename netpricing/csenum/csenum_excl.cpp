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

double csenum_excl::get_best_obj()
{
	return context.get_best_obj();
}

double csenum_excl::get_best_bound()
{
	return context.get_best_bound();
}

double csenum_excl::get_gap()
{
	return context.get_gap_ratio();
}

int csenum_excl::get_step_count()
{
	return context.step_count;
}