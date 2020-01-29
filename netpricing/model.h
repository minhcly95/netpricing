#pragma once

#include "problem.h"
#include "problem_multi.h"
#include "solution.h"

#include <utility>
#include <ilcplex/ilocplex.h>

struct model_base {
	double time;

	virtual bool solve() = 0;

	virtual void end() { }

	double getTime() {
		return time;
	}

	virtual solution get_solution() = 0;

	virtual std::string get_report() = 0;
};

struct model_single {
	using problem_type = problem;

	problem prob;
	int K, V, A, A1, A2;

	model_single(const problem& _prob) : prob(_prob) {
		K = prob.commodities.size();
		V = boost::num_vertices(prob.graph);
		A = boost::num_edges(prob.graph);
		A1 = prob.tolled_index_map.size();
		A2 = prob.tollfree_index_map.size();
	}
};

struct model_multi {
	using problem_type = problem_multi;

	problem_multi prob;
	int K, V, A, A1;
	std::vector<int> A2;

	model_multi(const problem_multi& _prob) : prob(_prob) {
		K = prob.commodities.size();
		V = boost::num_vertices(prob.graphs[0]);
		A = prob.max_edge_index;
		A1 = prob.tolled_index_common_map.size();

		for (auto& map : prob.tollfree_index_maps) {
			A2.push_back(map.size());
		}
	}
};

struct cplex_def {
	using NumVarArray = IloNumVarArray;
	using NumVarMatrix = IloArray<NumVarArray>;

	using NumArray = IloNumArray;
	using NumMatrix = IloArray<NumArray>;

	using RangeArray = IloRangeArray;
	using RangeMatrix = IloArray<RangeArray>;
	using ConstraintArray = IloConstraintArray;
};
