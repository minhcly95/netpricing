#pragma once

#include <ilcplex/ilocplex.h>

#include "../problem.h"
#include "../solution.h"

struct model {
	using NumVarArray = IloNumVarArray;
	using NumVarMatrix = IloArray<NumVarArray>;

	using NumArray = IloNumArray;
	using NumMatrix = IloArray<NumArray>;

	using RangeArray = IloRangeArray;
	using RangeMatrix = IloArray<RangeArray>;
	using ConstraintArray = IloConstraintArray;

	problem prob;
	IloEnv env;
	IloModel cplex_model;
	IloCplex cplex;
	int K, V, A, A1, A2;

	model(IloEnv& env, const problem& _prob) : env(env), cplex_model(env), cplex(cplex_model), prob(_prob) {
		K = prob.commodities.size();
		V = boost::num_vertices(prob.graph);
		A = boost::num_edges(prob.graph);
		A1 = prob.tolled_index_map.size();
		A2 = prob.tollfree_index_map.size();
	}

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

struct model_with_callbacks : public model {
	std::vector<IloCplex::Callback> callbacks;

	model_with_callbacks(IloEnv& env, const problem& _prob) : model(env, _prob) {}

	virtual bool solve() override {
		callbacks = attach_callbacks();
		return model::solve();
	}

	virtual void end() override {
		for (auto& cb : callbacks)
			cb.end();
		model::end();
	}

	virtual std::vector<IloCplex::Callback> attach_callbacks() = 0;
};

struct model_with_callback : public model_with_callbacks {
	IloCplex::Callback callback;

	model_with_callback(IloEnv& env, const problem& _prob) : model_with_callbacks(env, _prob) {}

	virtual IloCplex::Callback attach_callback() = 0;

	virtual std::vector<IloCplex::Callback> attach_callbacks() override {
		return std::vector<IloCplex::Callback>{ attach_callback() };
	}
};

