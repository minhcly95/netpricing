#include "tolls_heuristic.h"

#include <chrono>

#include "../macros.h"

using namespace std;

tolls_heuristic::tolls_heuristic(IloEnv& env, const problem& prob) :
	time(0), solver_f(prob), solver_i(env, prob)
{
}

solution tolls_heuristic::solve(const std::vector<cost_type>& tolls)
{
	auto start = chrono::high_resolution_clock::now();

	auto sol = solve_impl(tolls);

	auto end = chrono::high_resolution_clock::now();
	time += chrono::duration<double>(end - start).count();

	return sol;
}

solution tolls_heuristic::solve_impl(const std::vector<cost_type>& tolls)
{
	auto paths = solver_f.solve(tolls);
	auto new_tolls = solver_i.solve(paths);

	solution sol;
	sol.paths = std::move(paths);
	sol.tolls = std::move(new_tolls);

	return sol;
}
