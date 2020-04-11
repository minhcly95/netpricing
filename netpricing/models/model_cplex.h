#pragma once

#include "../model.h"

struct model_cplex : public model_base, public cplex_def {
	IloEnv env;
	IloModel cplex_model;
	IloCplex cplex;

	model_cplex(IloEnv& env);

	virtual IloCplex get_cplex();

	virtual void presolve() {}
	virtual bool solve_impl() override;
	virtual void config(const model_config& conf) override;
	virtual void end() override;

	virtual double get_best_obj() override;
	virtual double get_best_bound() override;
	virtual double get_gap() override;
	virtual int get_step_count() override;
};

struct model_with_callbacks : public model_cplex {
	std::vector<IloCplex::Callback> callbacks;

	model_with_callbacks(IloEnv& env);

	virtual bool solve_impl() override;
	virtual void end() override;

	virtual std::vector<IloCplex::Callback> attach_callbacks() = 0;
};

struct model_with_callback : public model_with_callbacks {
	IloCplex::Callback callback;

	model_with_callback(IloEnv& env);

	virtual IloCplex::Callback attach_callback() = 0;

	virtual std::vector<IloCplex::Callback> attach_callbacks() override;
};

struct model_with_generic_callbacks : public model_cplex {
	using ContextId = CPXLONG;

	std::vector<std::pair<IloCplex::Callback::Function*, ContextId>> callbacks;

	model_with_generic_callbacks(IloEnv& env);

	virtual bool solve_impl() override;
	virtual void end() override;

	virtual std::vector<std::pair<IloCplex::Callback::Function*, ContextId>> attach_callbacks() = 0;
};

struct model_with_goal : public model_cplex {
	IloCplex::Goal goal;
	double goal_time;

	model_with_goal(IloEnv& env, IloCplex::Goal goal);

	virtual bool solve_impl() override;

	virtual std::string get_report() override;
};
