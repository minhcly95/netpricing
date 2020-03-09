#include "inverse_solver.h"

#include <chrono>

#include "../macros.h"

using namespace std;

inverse_solver::inverse_solver(IloEnv& _env, const problem& _prob) :
	model_single(_prob), time(0),
	env(_env), cplex_model(env), cplex(cplex_model)
{
	// Typedef
	using graph_type = problem::graph_type;

	// Variables
	lambda = VarMatrix(env, K);
	t = VarArray(env, A1, 0, IloInfinity);

	LOOP(k, K) lambda[k] = NumVarArray(env, V, -IloInfinity, IloInfinity);

	// Objective (depending on path)
	obj = IloMaximize(env);

	// Dual feasibility (lower bound depends on path)
	dual_feas = RangeMatrix(env, K);
	LOOP(k, K) dual_feas[k] = RangeArray(env, A);
	LOOP(a, A) {
		SRC_DST_FROM_A(prob, a);
		bool is_tolled = prob.is_tolled_map[edge];
		cost_type cost = prob.cost_map[edge];

		LOOP(k, K) {
			dual_feas[k][a] = IloRange(lambda[k][src] - lambda[k][dst] <= cost);
			if (is_tolled)
				dual_feas[k][a].setLinearCoef(t[A_TO_A1(prob, a)], -1);
		}
	}

	// Add to model
	LOOP(k, K) cplex_model.add(lambda[k]);
	cplex_model.add(t);
	cplex_model.add(obj);
	LOOP(k, K) cplex_model.add(dual_feas[k]);

	cplex.setOut(env.getNullStream());
}

std::vector<cost_type> inverse_solver::solve(const std::vector<path>& paths)
{
	auto start = chrono::high_resolution_clock::now();

	auto tolls = solve_impl(paths);

	auto end = chrono::high_resolution_clock::now();
	time += chrono::duration<double>(end - start).count();

	return tolls;
}

std::vector<cost_type> inverse_solver::solve_impl(const std::vector<path>& paths)
{
	// Clear old states
	NumArray tcoefs(env, A1);
	LOOP(k, K) LOOP(a, A) {
		dual_feas[k][a].setLB(-IloInfinity);
	}

	// Set new states
	LOOP(k, K) {
		demand_type demand = prob.commodities[k].demand;
		const path& path = paths[k];

		for (int i = 0; i < path.size() - 1; i++) {
			auto edge = EDGE_FROM_SRC_DST(prob, path[i], path[i + 1]);
			int a = EDGE_TO_A(prob, edge);
			dual_feas[k][a].setLB(dual_feas[k][a].getUB());

			if (prob.is_tolled_map[edge]) {
				int a1 = EDGE_TO_A1(prob, edge);
				tcoefs[a1] += demand;
			}
		}
	}

	obj.setLinearCoefs(t, tcoefs);

	// Solve
	cplex.solve();

	// Extract result
	vector<cost_type> tolls(A1);
	NumArray tvals(env);

	cplex.getValues(t, tvals);
	LOOP(a, A1) tolls[a] = tvals[a];

	tcoefs.end();
	tvals.end();

	return tolls;
}
