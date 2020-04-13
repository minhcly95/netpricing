#include "hybrid_model.h"

#include "formulation.h"
#include "../../macros.h"
#include "../../utilities/set_var_name.h"

#include <iostream>

using namespace std;

struct hybrid_callback : public IloCplex::Callback::Function {
	hybrid_model& model;

	hybrid_callback(hybrid_model& model)
		: model(model) {
	}

	virtual void invoke(const IloCplex::Callback::Context& context) override {
		auto start = std::chrono::high_resolution_clock::now();

		// Extract t values
		IloNumArray tvals(model.env, model.A1);
		context.getCandidatePoint(model.t, tvals);

		// Invocation
		for (formulation* f : model.all_formulations) {
			if (f->has_callback())
				f->invoke_callback(context, tvals);
		}

		// Post heuristic
		if (all_of(model.all_formulations.begin(), model.all_formulations.end(),
				   [](formulation* f) {
					   return f->has_callback_solution();
				   })) {

			double obj = 0;
			for (formulation* f : model.all_formulations)
				obj += f->get_callback_obj();

			// Post solution if better
			if (obj > context.getIncumbentObjective()) {
				vector<pair<IloNumVar, IloNum>> all_pairs;

				// Add values of T
				LOOP(a, model.A1) all_pairs.emplace_back(model.t[a], tvals[a]);

				// Append sub-solutions
				for (formulation* f : model.all_formulations) {
					auto f_pairs = f->get_callback_solution(tvals);
					all_pairs.insert(all_pairs.end(),
									 make_move_iterator(f_pairs.begin()),
									 make_move_iterator(f_pairs.end()));
				}

				// Convert to Cplex format
				IloNumVarArray all_vars(model.env, all_pairs.size());
				IloNumArray all_vals(model.env, all_pairs.size());

				LOOP(i, all_pairs.size()) {
					all_vars[i] = all_pairs[i].first;
					all_vals[i] = all_pairs[i].second;
				}

				context.postHeuristicSolution(all_vars, all_vals, obj,
											  IloCplex::Callback::Context::SolutionStrategy::NoCheck);

				all_vars.end();
				all_vals.end();
			}
		}

		// Clean up
		tvals.end();

		auto end = std::chrono::high_resolution_clock::now();
		model.cb_time += std::chrono::duration<double>(end - start).count();
		model.cb_count++;
	}
};

hybrid_model::hybrid_model(IloEnv& env, const problem& _prob) :
	model_with_generic_callback(env), model_single(_prob),
	t(env, A1, 0, IloInfinity), cb_time(0), cb_count(0)
{
	SET_VAR_NAMES(*this, t);
	obj = IloMaximize(env);

	cplex_model.add(obj);
	cplex_model.add(t);
}

hybrid_model::~hybrid_model()
{
	for (auto f : all_formulations) {
		delete f;
	}
}

void hybrid_model::formulate()
{
	all_formulations = assign_formulations();
	LOOP(k, K) {
		all_formulations[k]->formulate(this, k);
	}
}

bool hybrid_model::solve_impl()
{
	cout << "Formulating..." << endl;
	formulate();
	auto form_done = std::chrono::high_resolution_clock::now();
	double form_time = std::chrono::duration<double>(form_done - start_time).count();
	cout << "Formulation done in " << form_time  << " s" << endl << endl;

	return model_with_generic_callback::solve_impl();
}

solution hybrid_model::get_solution()
{
	return solution();
}

pair<IloCplex::Callback::Function*, hybrid_model::ContextId> hybrid_model::attach_callback()
{
	if (any_of(all_formulations.begin(), all_formulations.end(), [](formulation* f) { return f->has_callback(); }))
		return make_pair(new hybrid_callback(*this), CPX_CALLBACKCONTEXT_CANDIDATE);
	else
		return make_pair(nullptr, 0);
}

string hybrid_model::get_report()
{
	ostringstream ss;
	ss << model_cplex::get_report();
	ss << "CALLBACK: " << cb_count <<
		"    Time " << cb_time << " s" <<
		"    Avg " << (cb_time * 1000 / cb_count) << " ms" << endl;
	return ss.str();
}
