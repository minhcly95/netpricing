#pragma once

#include <ilcplex/ilocplex.h>

#include "../problem.h"

struct model {
	problem prob;
	IloModel cplex_model;
	int K, V, A, A1, A2;

	model(IloEnv& env, const problem& _prob) : cplex_model(env), prob(_prob) {
		K = prob.commodities.size();
		V = boost::num_vertices(prob.graph);
		A = boost::num_edges(prob.graph);
		A1 = prob.tolled_index_map.size();
		A2 = prob.tollfree_index_map.size();
	}
};
