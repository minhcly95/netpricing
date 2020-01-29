#pragma once

#include <ilcplex/ilocplex.h>

#include "../model.h"

struct model_cplex : public model_base {
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

	model_cplex(IloEnv& env) : env(env), cplex_model(env), cplex(cplex_model) { }

	virtual IloCplex get_cplex() {
		return cplex;
	}

	virtual bool solve() override {
		IloNum begin = cplex.getTime();
		bool result = cplex.solve();
		time = cplex.getTime() - begin;
		return result;
	}

	virtual void end() override {
		cplex.end();
		cplex_model.end();
	}

	virtual solution get_solution() = 0;

	virtual std::string get_report() = 0;
};

struct model_with_callbacks : public model_cplex {
	std::vector<IloCplex::Callback> callbacks;

	model_with_callbacks(IloEnv& env) : model_cplex(env) {}

	virtual bool solve() override {
		callbacks = attach_callbacks();
		return model_cplex::solve();
	}

	virtual void end() override {
		for (auto& cb : callbacks)
			cb.end();
		model_cplex::end();
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

struct model_with_generic_callbacks : public model_cplex {
	using ContextId = CPXLONG;

	std::vector<std::pair<IloCplex::Callback::Function*, ContextId>> callbacks;

	model_with_generic_callbacks(IloEnv& env) : model_cplex(env) {}

	virtual bool solve() override {
		callbacks = attach_callbacks();
		for (auto& cb : callbacks)
			cplex.use(cb.first, cb.second);

		return model_cplex::solve();
	}

	virtual void end() override {
		for (auto& cb : callbacks)
			delete cb.first;
		model_cplex::end();
	}

	virtual std::vector<std::pair<IloCplex::Callback::Function*, ContextId>> attach_callbacks() = 0;
};

struct model_with_goal : public model_cplex {
	IloCplex::Goal goal;

	model_with_goal(IloEnv& env, IloCplex::Goal goal) : model_cplex(env), goal(goal) {}

	virtual bool solve() override {
		IloNum begin = cplex.getTime();
		bool result = cplex.solve(goal);
		time = cplex.getTime() - begin;
		return result;
	}
};
