#include "model_cplex.h"

#include <iostream>

using namespace std;

// model_cplex
model_cplex::model_cplex(IloEnv& env) :
	env(env), cplex_model(env), cplex(cplex_model), relaxation(0), relax_only(false) {
	cplex.setParam(IloCplex::ClockType, 2);
}

void model_cplex::solve_relaxation()
{
	cout << "Solving relaxation..." << endl;
	auto start = std::chrono::high_resolution_clock::now();

	IloModel relaxation_model(env);
	relaxation_model.add(cplex_model);

	// Get all the variables
	VarArray all_vars(env);
	for (IloModel::Iterator it(cplex_model); it.ok(); ++it) {
		if ((*it).isVariable())
			all_vars.add((*it).asVariable());
	}

	// Convert all variables to continuous
	IloConversion conv(env, all_vars, ILOFLOAT);
	relaxation_model.add(conv);

	// Solve the relaxed model
	IloCplex relaxation_cplex(relaxation_model);
	relaxation_cplex.setParam(IloCplex::Threads, 1);
	relaxation_cplex.setOut(env.getNullStream());
	relaxation_cplex.solve();
	relaxation = relaxation_cplex.getObjValue();

	auto end = std::chrono::high_resolution_clock::now();
	double time = std::chrono::duration<double>(end - start).count();
	cout << "Solving relaxation done in " << time << " s" << endl << endl;
}

IloCplex model_cplex::get_cplex() {
	return cplex;
}

bool model_cplex::solve_impl() {
	presolve();
	solve_relaxation();
	return relax_only ? true : cplex.solve();
}

void model_cplex::config(const model_config& conf) {
	cplex.setParam(IloCplex::Threads, conf.num_thread);
	cplex.setParam(IloCplex::Param::MIP::Strategy::VariableSelect, conf.var_select);
	if (conf.time_limit > 0) {
		cplex.setParam(IloCplex::Param::TimeLimit, conf.time_limit);
	}
	this->relax_only = conf.relax_only;
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

std::string model_cplex::get_report() {
	std::ostringstream ss;
	if (!relax_only)
		ss << model_base::get_report();
	ss << "RELAXATION: " << relaxation << std::endl;
	return ss.str();
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
	presolve();
	solve_relaxation();
	return relax_only ? true : cplex.solve(goal);
}

std::string model_with_goal::get_report() {
	std::ostringstream ss;
	ss << model_cplex::get_report();
	if (!relax_only)
		ss << "GOAL: " << goal_time << " s" << std::endl;
	return ss.str();
}
