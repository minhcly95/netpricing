#pragma once

#include <ilcplex/ilocplex.h>

#include "../problem.h"
#include "../problem_multi.h"
#include "../solution.h"

#include <utility>

struct model_base {
	using NumVarArray = IloNumVarArray;
	using NumVarMatrix = IloArray<NumVarArray>;

	using NumArray = IloNumArray;
	using NumMatrix = IloArray<NumArray>;

	using RangeArray = IloRangeArray;
	using RangeMatrix = IloArray<RangeArray>;
	using ConstraintArray = IloConstraintArray;

	IloEnv env;
	IloModel cplex_model;
	IloCplex cplex;

	model_base(IloEnv& env) : env(env), cplex_model(env), cplex(cplex_model) { }

	virtual IloCplex get_cplex() {
		return cplex;
	}

	virtual bool solve() {
		cplex.resetTime();
		return cplex.solve();
	}

	virtual void end() {
		cplex.end();
		cplex_model.end();
	}

	virtual solution get_solution() = 0;

	virtual std::string get_report() = 0;
};

struct model_with_callbacks : public model_base {
	std::vector<IloCplex::Callback> callbacks;

	model_with_callbacks(IloEnv& env) : model_base(env) {}

	virtual bool solve() override {
		callbacks = attach_callbacks();
		return model_base::solve();
	}

	virtual void end() override {
		for (auto& cb : callbacks)
			cb.end();
		model_base::end();
	}

	virtual std::vector<IloCplex::Callback> attach_callbacks() = 0;
};

struct model_with_callback : public model_with_callbacks {
	IloCplex::Callback callback;

	model_with_callback(IloEnv& env) : model_with_callbacks(env) {}

	virtual IloCplex::Callback attach_callback() = 0;

	virtual std::vector<IloCplex::Callback> attach_callbacks() override {
		return std::vector<IloCplex::Callback>{ attach_callback() };
	}
};

struct model_with_generic_callbacks : public model_base {
	using ContextId = CPXLONG;

	std::vector<std::pair<IloCplex::Callback::Function*, ContextId>> callbacks;

	model_with_generic_callbacks(IloEnv& env) : model_base(env) {}

	virtual bool solve() override {
		callbacks = attach_callbacks();
		for (auto& cb : callbacks)
			cplex.use(cb.first, cb.second);

		return model_base::solve();
	}

	virtual void end() override {
		for (auto& cb : callbacks)
			delete cb.first;
		model_base::end();
	}

	virtual std::vector<std::pair<IloCplex::Callback::Function*, ContextId>> attach_callbacks() = 0;
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
