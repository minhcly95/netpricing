#pragma once

#include "../model.h"

struct model_cplex : public model_base, public cplex_def {
	IloEnv env;
	IloModel cplex_model;
	IloCplex cplex;

	model_cplex(IloEnv& env) : env(env), cplex_model(env), cplex(cplex_model) {
		cplex.setParam(IloCplex::ClockType, 2);
	}

	virtual IloCplex get_cplex() {
		return cplex;
	}

	virtual void presolve() {}

	virtual bool solve_impl() override {
		presolve();
		return cplex.solve();
	}

	virtual void config(const model_config& conf) override {
		cplex.setParam(IloCplex::Threads, conf.num_thread);
		cplex.setParam(IloCplex::Param::MIP::Strategy::VariableSelect, conf.var_select);
		if (conf.time_limit > 0) {
			cplex.setParam(IloCplex::Param::TimeLimit, conf.time_limit);
		}
	}

	virtual void end() override {
		cplex.end();
		cplex_model.end();
	}

	virtual double get_best_obj() override {
		return cplex.getObjValue();
	}
	virtual double get_best_bound() override {
		return cplex.getBestObjValue();
	}
	virtual double get_gap() override {
		return cplex.getMIPRelativeGap();
	}
	virtual int get_step_count() override {
		return cplex.getNnodes();
	}
};

struct model_with_callbacks : public model_cplex {
	std::vector<IloCplex::Callback> callbacks;

	model_with_callbacks(IloEnv& env) : model_cplex(env) {}

	virtual bool solve_impl() override {
		callbacks = attach_callbacks();
		return model_cplex::solve_impl();
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

	virtual bool solve_impl() override {
		callbacks = attach_callbacks();
		for (auto& cb : callbacks)
			cplex.use(cb.first, cb.second);

		return model_cplex::solve_impl();
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
	double goal_time;

	model_with_goal(IloEnv& env, IloCplex::Goal goal) : model_cplex(env), goal(goal), goal_time(0) {}

	virtual bool solve_impl() override {
		return cplex.solve(goal);
	}

	virtual std::string get_report() override {
		std::ostringstream ss;
		ss << model_cplex::get_report();
		ss << "GOAL: " << goal_time << " s" << std::endl;
		return ss.str();
	}
};
