#pragma once

#include "../model.h"
#include "../graph/light_graph.h"
#include "csenum_node.h"

#include <queue>
#include <chrono>

struct csenum : public model_base, public model_single, public cplex_def {
	light_graph primal_lgraph;
	IloEnv env;
	IloModel dual_model;
	IloCplex dual_cplex;

	// Dual model
	VarMatrix lambda;
	VarArray t;
	IloObjective dual_obj;
	RangeMatrix dual_feas;

	// State management
	std::vector<int> primal_state_stack;
	std::vector<csenum_coor> dual_state_stack;

	// Enumeration
	std::priority_queue<csenum_node> queue;
	csenum_node best_node;

	// Statistics
	int node_count;
	int step_count;
	double time_limit;

	// Constructor
	csenum(IloEnv& env, const problem& prob);
	void build_dual_model();

	// Enumeration process
	void solve_root();
	void step();

	// State management
	void clear_primal_state();
	void clear_dual_state();

	void set_primal_state(std::vector<int> as);
	void set_dual_state(std::vector<csenum_coor> coors);

	void push_primal_state(int a);
	void push_dual_state(const csenum_coor& coor);

	void pop_primal_state();
	void pop_dual_state();

	// Query functions
	double get_best_obj();
	double get_best_bound();

	// Utilities
	void print_node(const csenum_node& node, bool feasible = false);

	// Inherited via model_base
	virtual bool solve_impr() override;
	virtual solution get_solution() override;
	virtual std::string get_report() override;
	virtual void config(const model_config& config) override;
};
