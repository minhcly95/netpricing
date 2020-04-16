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

	void callback_routine(const IloCplex::Callback::Context& context) {
		auto start = std::chrono::high_resolution_clock::now();

		// Extract t values
		IloNumArray tvals(model.env, model.A1);
		context.getCandidatePoint(model.t, tvals);

		// Invocation
		for (formulation* f : model.all_formulations) {
			if (f->has_callback())
				f->invoke_callback(context, tvals);
		}

		// Post heuristic if all formulations have callback optimal path or no callback
		if (all_of(model.all_formulations.begin(), model.all_formulations.end(),
				   [](formulation* f) {
					   return !f->has_callback() || f->has_callback_optimal_path();
				   })) {

			double obj = 0;
			for (formulation* f : model.all_formulations) {
				if (f->has_callback() && f->has_callback_optimal_path())
					obj += f->get_callback_obj();
				else {
					IloExpr expr = f->get_obj_expr();
					obj += context.getCandidateValue(expr);
					expr.end();
				}
			}

			// Post solution if better
			if (obj > context.getIncumbentObjective()) {
				vector<pair<IloNumVar, IloNum>> all_pairs;

				// Add values of T
				LOOP(a, model.A1) all_pairs.emplace_back(model.t[a], tvals[a]);

				// Append sub-solutions
				for (formulation* f : model.all_formulations) {
					// Extract directly if f has callback
					if (f->has_callback() && f->has_callback_optimal_path()) {
						vector<int> opt_path = f->get_callback_optimal_path();
						auto f_pairs = f->path_to_solution(tvals, opt_path);
						all_pairs.insert(all_pairs.end(),
										 make_move_iterator(f_pairs.begin()),
										 make_move_iterator(f_pairs.end()));
					}
					// Otherwise, extract from the current candidate
					else {
						auto vars = f->get_all_variables();
						int n = vars.size();
						IloNumVarArray cplex_vars(model.env, n);
						LOOP(i, n) cplex_vars[i] = vars[i];

						IloNumArray vals(model.env, n);
						context.getCandidatePoint(cplex_vars, vals);

						all_pairs.reserve(all_pairs.size() + n);
						LOOP(i, n) all_pairs.emplace_back(vars[i], vals[i]);

						cplex_vars.end();
						vals.end();
					}
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

	void heuristic_routine(const IloCplex::Callback::Context& context) {
		int node_count = context.getIntInfo(IloCplex::Callback::Context::Info::NodeCount);
		if (model.heur_freq == 0 || node_count % model.heur_freq > 0)
			return;

		auto start = std::chrono::high_resolution_clock::now();

		// Extract t values
		IloNumArray tvals(model.env, model.A1);
		context.getRelaxationPoint(model.t, tvals);

		vector<cost_type> tolls(model.A1);
		LOOP(a, model.A1) tolls[a] = tvals[a];

		// Solve for paths
		auto paths = model.heur_solver.solve(tolls);

		// Test for objective
		double obj = 0;
		LOOP(k, model.K) obj += model.heur_solver.lgraph.get_path_toll(paths[k]) * model.prob.commodities[k].demand / 0.9999;

		// Post solution if better
		if (obj > context.getIncumbentObjective()) {
			vector<pair<IloNumVar, IloNum>> all_pairs;

			// Add values of T
			LOOP(a, model.A1) all_pairs.emplace_back(model.t[a], tvals[a]);

			// Append sub-solutions
			LOOP(k, model.K) {
				auto f_pairs = model.all_formulations[k]->path_to_solution(tvals, paths[k]);
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

		// Add heuristic cut if it is supported
		LOOP(k, model.K) {
			formulation* f = model.all_formulations[k];
			if (f->has_heuristic_cut())
				f->post_heuristic_cut(context, paths[k]);
		}

		// Clean up
		tvals.end();

		auto end = std::chrono::high_resolution_clock::now();
		model.heur_time += std::chrono::duration<double>(end - start).count();
		model.heur_count++;
	}

	virtual void invoke(const IloCplex::Callback::Context& context) override {
		if (context.inCandidate())
			callback_routine(context);
		else
			heuristic_routine(context);
	}
};

hybrid_model::hybrid_model(IloEnv& env, const problem& _prob) :
	model_with_generic_callback(env), model_single(_prob),
	t(env, A1, 0, IloInfinity), heur_freq(100),
	cb_time(0), cb_count(0), heur_time(0), heur_count(0),
	heur_solver(prob)
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
	bool cb_enabled = any_of(all_formulations.begin(), all_formulations.end(),
							 [](formulation* f) { return f->has_callback(); });

	if (cb_enabled || heur_freq > 0)
		return make_pair(new hybrid_callback(*this),
						 (cb_enabled ? CPX_CALLBACKCONTEXT_CANDIDATE : 0) |
						 (heur_freq > 0 ? CPX_CALLBACKCONTEXT_RELAXATION : 0));
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
	ss << "HEURISTIC: " << heur_count <<
		"    Time " << heur_time << " s" <<
		"    Avg " << (heur_time * 1000 / heur_count) << " ms" << endl;
	return ss.str();
}

void hybrid_model::config(const model_config& conf)
{
	model_cplex::config(conf);
	if (conf.heur_freq >= 0)
		heur_freq = conf.heur_freq;
}
