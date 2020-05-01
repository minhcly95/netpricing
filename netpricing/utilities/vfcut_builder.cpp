#include "vfcut_builder.h"

#include <chrono>
#include "../macros.h"

using namespace std;

vfcut_builder::vfcut_builder(IloEnv& env, const problem& _prob,
							 const VarMatrix& x,
							 const VarMatrix& y,
							 const VarArray& t,
							 const VarMatrix& tx) :
	env(env), solver(_prob), prob(solver.prob), x(x), y(y), t(t), tx(tx),
	time(0), count(0)
{
}

void vfcut_builder::build(const NumArray& tvals, RangeArray& cuts, NumMatrix& xvals, NumMatrix& yvals, IloNum& obj)
{
	auto start = chrono::high_resolution_clock::now();

	// Prepare
	vector<cost_type> tolls(solver.A1);
	LOOP(a, solver.A1) tolls[a] = tvals[a];

	// Solve
	auto paths = solver.solve(tolls);

	// Extract
	obj = 0;
	LOOP(k, solver.K) {
		// Cut formulation
		cuts.add(build_cut(paths[k], k));

		// Solution formulation
		NumArray xk(env, solver.A1), yk(env, solver.A2);
		IloNum objk = 0;

		vector<int>& path = paths[k];
		for (int i = 0; i < path.size() - 1; i++) {
			auto edge = boost::edge(path[i], path[i + 1], prob.graph).first;
			if (prob.is_tolled_map[edge]) {
				int a1 = EDGE_TO_A1(prob, edge);
				xk[a1] = 1;
				objk += tvals[a1];
			}
			else {
				int a2 = EDGE_TO_A2(prob, edge);
				yk[a2] = 1;
			}
		}

		xvals.add(xk);
		yvals.add(yk);
		obj += objk * prob.commodities[k].demand;
	}

	auto end = chrono::high_resolution_clock::now();
	time += chrono::duration<double>(end - start).count();
	++count;
}

IloRange vfcut_builder::build_cut(const vector<int>& path, int k)
{
	// Cut formulation
	IloExpr cut_lhs(env);
	IloNum cut_rhs = 0;

	// Fixed part
	LOOP(a, solver.A1) {
		auto edge = A1_TO_EDGE(prob, a);
		cost_type cost = prob.cost_map[edge];
		cut_lhs.setLinearCoef(x[k][a], cost);
		cut_lhs.setLinearCoef(tx[k][a], 1);
	}
	LOOP(a, solver.A2) {
		auto edge = A2_TO_EDGE(prob, a);
		cost_type cost = prob.cost_map[edge];
		cut_lhs.setLinearCoef(y[k][a], cost);
	}

	// Path-depending part
	for (int i = 0; i < path.size() - 1; i++) {
		auto edge = boost::edge(path[i], path[i + 1], prob.graph).first;

		cut_rhs += prob.cost_map[edge];
		if (prob.is_tolled_map[edge]) {
			int a1 = EDGE_TO_A1(prob, edge);
			cut_lhs.setLinearCoef(t[a1], -1);
		}
	}

	return cut_lhs <= cut_rhs * (1 + TOLERANCE);
}

double vfcut_builder::get_sub_time() const { return solver.time; }

void vfcut_builder::reset() {
	time = 0;
	count = 0;
	solver.time = 0;
}
