#include "follower_cplex_solver.h"
#include "macros.h"

#include <algorithm>
#include <chrono>

using namespace std;
using namespace boost;

follower_cplex_solver::follower_cplex_solver(IloEnv& env, const problem& prob) :
	model_single(prob), env(env), cplex_model(env, K), cplex(env, K), time(0)
{
	// Typedef
	using graph_type = problem::graph_type;

	// Variables
	x = NumVarMatrix(env, K);
	y = NumVarMatrix(env, K);
	z = NumVarMatrix(env, K);

	LOOP(k, K) {
		x[k] = NumVarArray(env, A1, 0, IloInfinity);
		y[k] = NumVarArray(env, A2, 0, IloInfinity);
		z[k] = NumVarArray(env, A);
	}

	// z is used to reference x and y
	LOOP(a1, A1) {
		int a = A1_TO_A(prob, a1);
		LOOP(k, K) z[k][a] = x[k][a1];
	}
	LOOP(a2, A2) {
		int a = A2_TO_A(prob, a2);
		LOOP(k, K) z[k][a] = y[k][a2];
	}

	// Objective (only set for y)
	obj = IloArray<IloObjective>(env, K);
	LOOP(k, K) {
		obj[k] = IloMinimize(env);
		LOOP(a, A2) {
			auto edge = A2_TO_EDGE(prob, a);
			obj[k].setLinearCoef(y[k][a], prob.cost_map[edge]);
		}
	}

	// Flow constraints
	flow_constr = RangeMatrix(env, K);
	LOOP(k, K) {
		flow_constr[k] = RangeArray(env, V, 0, 0);

		// Supply and demand
		flow_constr[k][prob.commodities[k].origin].setBounds(1, 1);
		flow_constr[k][prob.commodities[k].destination].setBounds(-1, -1);
	}

	// Flow matrix
	LOOP(a, A) {
		SRC_DST_FROM_A(prob, a)
		LOOP(k, K) flow_constr[k][src].setLinearCoef(z[k][a], 1);
		LOOP(k, K) flow_constr[k][dst].setLinearCoef(z[k][a], -1);
	}

	// Add to model
	LOOP(k, K) {
		cplex_model[k] = IloModel(env);
		cplex_model[k].add(obj[k]);
		cplex_model[k].add(flow_constr[k]);
		cplex[k] = IloCplex(cplex_model[k]);
		cplex[k].setOut(env.getNullStream());
	}
}

vector<follower_cplex_solver::path> follower_cplex_solver::solve(const vector<cost_type>& tolls)
{
	auto start = chrono::high_resolution_clock::now();

	// Add toll
	LOOP(k, K) LOOP(a, A1) {
		auto edge = A1_TO_EDGE(prob, a);
		obj[k].setLinearCoef(x[k][a], prob.cost_map[edge] + tolls[a]);
	}

	// Solve
	LOOP(k, K) {
		cplex[k].solve();
	}

	// Extract result
	vector<path> paths(K);
	NumArray zvals(env);

	LOOP(k, K) {
		cplex[k].getValues(z[k], zvals);

		// Map src -> dst
		map<int, int> src_dst_map;
		LOOP(a, A) {
			if (zvals[a] > 0.5) {
				SRC_DST_FROM_A(prob, a);
				src_dst_map[src] = dst;
			}
		}

		// Trace the path
		int curr = prob.commodities[k].origin;
		int dest = prob.commodities[k].destination;
		while (curr != dest) {
			paths[k].push_back(curr);
			curr = src_dst_map[curr];
		}
		paths[k].push_back(dest);
	}

	zvals.end();

	auto end = chrono::high_resolution_clock::now();
	time += chrono::duration<double>(end - start).count();

	return paths;
}

follower_cplex_solver::~follower_cplex_solver()
{
	cplex.end();
	cplex_model.end();

	LOOP(k, K) {
		x[k].end();
		y[k].end();
		flow_constr[k].end();
		obj[k].end();
	}
	x.end();
	y.end();
	z.end();
	flow_constr.end();
	obj.end();
}
