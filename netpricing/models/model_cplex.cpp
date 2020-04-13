#include "model_cplex.h"

// model_cplex
model_cplex::model_cplex(IloEnv& env) : env(env), cplex_model(env), cplex(cplex_model) {
	cplex.setParam(IloCplex::ClockType, 2);
}

IloCplex model_cplex::get_cplex() {
	return cplex;
}

bool model_cplex::solve_impl() {
	presolve();
	return cplex.solve();
}

void model_cplex::config(const model_config& conf) {
	cplex.setParam(IloCplex::Threads, conf.num_thread);
	cplex.setParam(IloCplex::Param::MIP::Strategy::VariableSelect, conf.var_select);
	if (conf.time_limit > 0) {
		cplex.setParam(IloCplex::Param::TimeLimit, conf.time_limit);
	}
}

void model_cplex::end() {
	cplex.end();
	cplex_model.end();
}

double model_cplex::get_best_obj() {
	return cplex.getObjValue();
}

double model_cplex::get_best_bound() {
	return cplex.getBestObjValue();
}

double model_cplex::get_gap() {
	return cplex.getMIPRelativeGap();
}

int model_cplex::get_step_count() {
	return cplex.getNnodes();
}

// model_with_callbacks
model_with_callbacks::model_with_callbacks(IloEnv& env) : model_cplex(env) {}

bool model_with_callbacks::solve_impl() {
	callbacks = attach_callbacks();
	return model_cplex::solve_impl();
}

void model_with_callbacks::end() {
	for (auto& cb : callbacks)
		cb.end();
	model_cplex::end();
}

// model_with_callback
model_with_callback::model_with_callback(IloEnv& env) : model_with_callbacks(env) {}

std::vector<IloCplex::Callback> model_with_callback::attach_callbacks() {
	return std::vector<IloCplex::Callback>{ attach_callback() };
}

// model_with_generic_callbacks
model_with_generic_callback::model_with_generic_callback(IloEnv& env) : model_cplex(env) {}

bool model_with_generic_callback::solve_impl() {
	callback = attach_callback();
	if (callback.first != nullptr)
		cplex.use(callback.first, callback.second);

	return model_cplex::solve_impl();
}

void model_with_generic_callback::end() {
	delete callback.first;
	model_cplex::end();
}

// model_with_goal
model_with_goal::model_with_goal(IloEnv& env, IloCplex::Goal goal) : model_cplex(env), goal(goal), goal_time(0) {}

bool model_with_goal::solve_impl() {
	return cplex.solve(goal);
}

std::string model_with_goal::get_report() {
	std::ostringstream ss;
	ss << model_cplex::get_report();
	ss << "GOAL: " << goal_time << " s" << std::endl;
	return ss.str();
}
