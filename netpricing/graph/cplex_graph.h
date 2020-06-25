#pragma once

#include <ilcplex/ilocplex.h>
#include <map>
#include <tuple>

#include "light_graph.h"
#include "../model.h"

struct cplex_graph : public light_graph, public cplex_def
{
	using path = std::vector<int>;

	IloEnv env;
	IloModel model;
	IloCplex cplex;

	NumVarArray x;

	IloObjective obj;
	RangeArray flow_constr;

	cplex_graph(const problem_base::graph_type& graph);

	path shortest_path_cplex(int from, int to);

	void reset();
	void force(int src, int dst);
	void disable(int src, int dst);

	std::vector<path> bilevel_feasible_paths_3(int from, int to, int k, bool filter=true);
};
