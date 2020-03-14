#include "value_func_model.h"

#include "../macros.h"
#include "model_utils.h"

#include <iostream>
#include <sstream>
#include <chrono>

using namespace std;
using namespace boost;

struct value_func_model_callback : public IloCplex::Callback::Function {
	value_func_model& m;
	double tol;
	model_cplex::NumVarArray heur_vars;

	value_func_model_callback(value_func_model& _m)
		: m(_m), tol(1e-6), heur_vars(m.get_cplex().getEnv()) {

		LOOP(k, m.K) heur_vars.add(m.x[k]);
		LOOP(k, m.K) heur_vars.add(m.y[k]);
		heur_vars.add(m.t);
		LOOP(k, m.K) heur_vars.add(m.tx[k]);
	}

	virtual ~value_func_model_callback() {
		heur_vars.end();
	}

	virtual void invoke(const IloCplex::Callback::Context& context) override {
		using NumArray = value_func_model::NumArray;
		using NumMatrix = value_func_model::NumMatrix;
		using RangeArray = value_func_model::RangeArray;

		IloEnv env = context.getEnv();
		bool is_candidate = context.inCandidate();

		// Only run heuristic every 100 nodes
		if (!is_candidate) {
			int node_count = context.getIntInfo(IloCplex::Callback::Context::Info::NodeCount);
			if (m.heur_freq == 0 || node_count % m.heur_freq > 0)
				return;
		}

		// Extract t values
		NumArray tvals(env, m.A1);
		if (is_candidate)
			context.getCandidatePoint(m.t, tvals);
		else
			context.getRelaxationPoint(m.t, tvals);

		// Separation algorithm
		RangeArray cuts(env);
		NumMatrix xvals(env), yvals(env);
		IloNum obj;

		m.builder.build(tvals, cuts, xvals, yvals, obj);

		// Add the cuts
		bool post_heur = true;
		if (is_candidate) {
			// Add the cuts when candidate is rejected
			bool rejected = false;
			LOOP(i, cuts.getSize()) {
				auto expr = cuts[i].getExpr();
				double lb = cuts[i].getLB();
				double ub = cuts[i].getUB();

				double expr_val = context.getCandidateValue(expr);
				if (!(lb * (1 - tol) <= expr_val && expr_val <= ub * (1 + tol))) {
					context.rejectCandidate(cuts);
					rejected = true;
					post_heur = false;
					break;
				}
			}
			//cout << (rejected ? "R" : "A");
		}
		else {
			// Add the cuts anyway
			LOOP(i, cuts.getSize())
				context.addUserCut(cuts[i], IloCplex::CutManagement::UseCutFilter, false);
			//cout << "C " << context.getIntInfo(IloCplex::Callback::Context::Info::NodeCount) << endl;
		}

		// Post heuristic solution
		if (post_heur && obj > context.getIncumbentObjective()) {
			NumArray heur_vals(env);

			LOOP(k, m.K) heur_vals.add(xvals[k]);
			LOOP(k, m.K) heur_vals.add(yvals[k]);
			heur_vals.add(tvals);
			LOOP(k, m.K) LOOP(a, m.A1) heur_vals.add(tvals[a] * xvals[k][a]);

			context.postHeuristicSolution(heur_vars, heur_vals, obj, IloCplex::Callback::Context::SolutionStrategy::NoCheck);

			//cout << "P";
			heur_vals.end();
		}

		// Clean up
		LOOP(i, cuts.getSize()) cuts[i].end();
		cuts.end();
		clean_up(xvals);
		clean_up(yvals);
		tvals.end();
	}
};

value_func_model::value_func_model(IloEnv& env, const problem& _prob) :
	model_with_generic_callbacks(env), model_single(_prob),
	x(env, K), y(env, K), z(env, K), tx(env, K), t(env, A1, 0, IloInfinity),
	builder(env, prob, x, y, t, tx), heur_freq(100), pre_cut(0),
	presolve_time(0), presolve_cut_count(0) {

	// Typedef
	using namespace std;
	using namespace boost;
	using graph_type = problem::graph_type;

	// Variables
	LOOP(k, K) {
		x[k] = NumVarArray(env, A1, 0, 1, ILOBOOL);
		y[k] = NumVarArray(env, A2, 0, IloInfinity);
		z[k] = NumVarArray(env, A);
		tx[k] = NumVarArray(env, A1, 0, IloInfinity);
	}

	// z is used to reference x and y
	LOOP(a1, A1) {
		int a = A1_TO_A(prob, a1);
		LOOP(k, K) z[k][a] = x[k][a1];
	}
	LOOP(a2, A2) {
		int a = A2_TO_A(prob, a2);
		LOOP(k, K) z[k][a] = y[k][a2];
	}

	// Objective
	obj = IloMaximize(env);
	LOOP(k, K) LOOP(a, A1) obj.setLinearCoef(tx[k][a], prob.commodities[k].demand);

	// Flow constraints
	flow_constr = RangeMatrix(env, K);
	LOOP(k, K) {
		flow_constr[k] = RangeArray(env, V, 0, 0);

		// Supply and demand
		flow_constr[k][prob.commodities[k].origin].setBounds(1, 1);
		flow_constr[k][prob.commodities[k].destination].setBounds(-1, -1);
	}

	// Flow matrix
	LOOP(a, A) {
		SRC_DST_FROM_A(prob, a);
		LOOP(k, K) flow_constr[k][src].setLinearCoef(z[k][a], 1);
		LOOP(k, K) flow_constr[k][dst].setLinearCoef(z[k][a], -1);
	}

	// Bilinear 1
	bilinear1 = RangeMatrix(env, K);
	LOOP(k, K) {
		bilinear1[k] = RangeArray(env, A1, -IloInfinity, 0);
		LOOP(a, A1) {
			bilinear1[k][a].setLinearCoef(tx[k][a], 1);
			bilinear1[k][a].setLinearCoef(x[k][a], -prob.big_m[k][a]);
		}
	}

	// Bilinear 2
	bilinear2 = RangeMatrix(env, K);
	LOOP(k, K) {
		bilinear2[k] = RangeArray(env, A1);
		LOOP(a, A1) {
			bilinear2[k][a] = IloRange(env, -IloInfinity, prob.big_n[a]);
			bilinear2[k][a].setLinearCoef(t[a], 1);
			bilinear2[k][a].setLinearCoef(tx[k][a], -1);
			bilinear2[k][a].setLinearCoef(x[k][a], prob.big_n[a]);
		}
	}

	// Bilinear 3
	bilinear3 = RangeMatrix(env, K);
	LOOP(k, K) {
		bilinear3[k] = RangeArray(env, A1, -IloInfinity, 0);
		LOOP(a, A1) {
			bilinear3[k][a].setLinearCoef(t[a], -1);
			bilinear3[k][a].setLinearCoef(tx[k][a], 1);
		}
	}

	// Add to model
	cplex_model.add(obj);
	LOOP(k, K) {
		cplex_model.add(flow_constr[k]);
		cplex_model.add(bilinear1[k]);
		cplex_model.add(bilinear2[k]);
		cplex_model.add(bilinear3[k]);
	}

	init_variable_name();
}

void value_func_model::init_variable_name() {
	LOOP(k, K) {
		LOOP(a, A1) {
			SRC_DST_FROM_A1(prob, a);
			char name[50];
			sprintf(name, "x[%d,%d->%d]", k, src, dst);
			x[k][a].setName(name);
		}

		LOOP(a, A2) {
			SRC_DST_FROM_A2(prob, a);
			char name[50];
			sprintf(name, "y[%d,%d->%d]", k, src, dst);
			y[k][a].setName(name);
		}

		LOOP(a, A1) {
			SRC_DST_FROM_A1(prob, a);
			char name[50];
			sprintf(name, "tx[%d,%d->%d]", k, src, dst);
			tx[k][a].setName(name);
		}
	}

	LOOP(a, A1) {
		SRC_DST_FROM_A1(prob, a);
		char name[50];
		sprintf(name, "t[%d->%d]", src, dst);
		t[a].setName(name);
	}
}

solution value_func_model::get_solution()
{
	NumMatrix zvals = get_values(cplex, z);
	NumArray tvals = get_values(cplex, t);

	solution sol = fetch_solution_from_z_t(*this, zvals, tvals);

	clean_up(zvals);
	tvals.end();

	return sol;
}

std::string value_func_model::get_report()
{
	ostringstream ss;
	ss << model_cplex::get_report();
	ss <<
		"PRESOLVE: " << presolve_time << " s" <<
		"    Count " << presolve_cut_count << endl <<
		"SEP: " << builder.count <<
		"    Time " << builder.time << " s" <<
		"    Avg " << (builder.time * 1000 / builder.count) << " ms" <<
		"    Sub " << (builder.get_sub_time() * 100 / builder.time) << "%" << endl;

	return ss.str();
}

vector<pair<IloCplex::Callback::Function*, value_func_model::ContextId>> value_func_model::attach_callbacks()
{
	return {
		make_pair(new value_func_model_callback(*this), CPX_CALLBACKCONTEXT_CANDIDATE | CPX_CALLBACKCONTEXT_RELAXATION)
	};
}

void value_func_model::presolve()
{
	if (pre_cut == 0)
		return;

	auto start = std::chrono::high_resolution_clock::now();

	LOOP(k, K) {
		int from = prob.commodities[k].origin;
		int to = prob.commodities[k].destination;

		vector<vector<int>> paths = builder.solver.lgraph.toll_unique_paths(from, to, pre_cut);

		for (auto& path : paths) {
			IloRange cut = builder.build_cut(path, k);
			cplex_model.add(cut);
		}

		presolve_cut_count += paths.size();
	}

	auto end = std::chrono::high_resolution_clock::now();
	presolve_time = std::chrono::duration<double>(end - start).count();

	builder.reset();
}

void value_func_model::config(const model_config& conf)
{
	model_cplex::config(conf);
	if (conf.heur_freq >= 0)
		heur_freq = conf.heur_freq;
	pre_cut = conf.pre_cut;
}
