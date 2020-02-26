#pragma once

#include "csenum_solver.h"
#include "csenum_node.h"

#include <queue>
#include <chrono>

struct csenum : public model_base, public cplex_def {
	using problem_type = problem;

	IloEnv env;
	csenum_solver solver;

	problem& prob;
	int K, V, A, A1, A2;

	// Enumeration
	std::priority_queue<csenum_node> queue;
	csenum_node best_node;

	// Statistics
	int node_count;
	int step_count;
	double time_limit;

	// Constructor
	csenum(IloEnv& env, const problem& prob);

	// Enumeration process
	void solve_root();
	void step();

	// Query functions
	double get_best_obj();
	double get_best_bound();

	// Utilities
	void print_node(const csenum_node& node, bool feasible = false);

	// Inherited via model_base
	virtual bool solve_impl() override;
	virtual solution get_solution() override;
	virtual std::string get_report() override;
	virtual void config(const model_config& config) override;
};
