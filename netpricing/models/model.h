#pragma once

#include <ilcplex/ilocplex.h>

#include "../problem.h"

struct model {
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

	virtual std::string get_report() = 0;
};

struct model_with_callback : public model {
	IloCplex::Callback callback;

	model_with_callback(IloEnv& env, const problem& _prob) : model(env, _prob) {}

	virtual bool solve() override {
		callback = attach_callback();
		return model::solve();
	}

	virtual void end() override {
		callback.end();
		model::end();
	}

	virtual IloCplex::Callback attach_callback() = 0;
};

