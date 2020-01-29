#include "value_func_model.h"

#include "../macros.h"
#include "model_utils.h"

#include <iostream>
#include <sstream>
#include <chrono>

#include <boost/graph/dijkstra_shortest_paths.hpp>

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

		// Only run every 100 nodes
		if (!is_candidate) {
			int node_count = context.getIntInfo(IloCplex::Callback::Context::Info::NodeCount);
			if (node_count % 100 > 0)
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

		m.separate(tvals, cuts, xvals, yvals, obj);

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
	separate_time(0), subprob_time(0), separate_count(0) {

	// Typedef
	using namespace std;
	using namespace boost;
	using graph_type = problem::graph_type;

	// Variables
	x = NumVarMatrix(env, K);
	y = NumVarMatrix(env, K);
	z = NumVarMatrix(env, K);
	tx = NumVarMatrix(env, K);
	t = NumVarArray(env, A1, 0, IloInfinity);

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
			char name[20];
			sprintf(name, "x[%d,%d->%d]", k, src, dst);
			x[k][a].setName(name);
		}

		LOOP(a, A2) {
			SRC_DST_FROM_A2(prob, a);
			char name[20];
			sprintf(name, "y[%d,%d->%d]", k, src, dst);
			y[k][a].setName(name);
		}

		LOOP(a, A1) {
			SRC_DST_FROM_A1(prob, a);
			char name[20];
			sprintf(name, "tx[%d,%d->%d]", k, src, dst);
			tx[k][a].setName(name);
		}
	}

	LOOP(a, A1) {
		SRC_DST_FROM_A1(prob, a);
		char name[20];
		sprintf(name, "t[%d->%d]", src, dst);
		t[a].setName(name);
	}
}

void value_func_model::separate(const NumArray& tvals,
								RangeArray& cuts, NumMatrix& xvals, NumMatrix& yvals, IloNum& obj) {
	auto start = chrono::high_resolution_clock::now();

	separate_inner(tvals, cuts, xvals, yvals, obj);

	auto end = chrono::high_resolution_clock::now();
	separate_time += chrono::duration<double>(end - start).count();
	++separate_count;
}

void value_func_model::separate_inner(const NumArray& tvals,
									  RangeArray& cuts, NumMatrix& xvals, NumMatrix& yvals, IloNum& obj)
{
	// Copy a new cost map
	using cost_map_type = map<problem::edge_descriptor, cost_type>;
	cost_map_type tolled_cost_map;

	problem::edge_iterator ei, ei_end;
	for (tie(ei, ei_end) = edges(prob.graph); ei != ei_end; ++ei)
		tolled_cost_map[*ei] = prob.cost_map[*ei];

	// Add toll to new cost map
	LOOP(a, A1) {
		auto edge = A1_TO_EDGE(prob, a);
		tolled_cost_map[edge] += tvals[a] * 0.9999;	// Prefer tolled arcs
	}

	// Objective value
	obj = 0;

	LOOP(k, K) {
		// Find the shortest path
		vector<int> parents(V);
		auto index_map = get(vertex_index, prob.graph);

		auto substart = chrono::high_resolution_clock::now();

		boost::dijkstra_shortest_paths(prob.graph, prob.commodities[k].origin,
									   weight_map(make_assoc_property_map(tolled_cost_map)).
									   predecessor_map(make_iterator_property_map(parents.begin(), index_map)));

		auto subend = chrono::high_resolution_clock::now();
		subprob_time += chrono::duration<double>(subend - substart).count();

		// Cut formulation
		IloExpr cut_lhs(cplex_model.getEnv());
		IloNum cut_rhs = 0;

		// Solution formulation
		NumArray xk(env, A1), yk(env, A2);
		IloNum objk = 0;

		// Fixed part
		LOOP(a, A) {
			auto edge = A_TO_EDGE(prob, a);
			cost_type cost = prob.cost_map[edge];
			cut_lhs.setLinearCoef(z[k][a], cost);
		}
		LOOP(a, A1) cut_lhs.setLinearCoef(tx[k][a], 1);

		// Path-depending part
		int current = prob.commodities[k].destination;
		while (current != prob.commodities[k].origin) {
			int prev = parents[current];
			auto edge = boost::edge(prev, current, prob.graph).first;

			cut_rhs += prob.cost_map[edge];
			if (prob.is_tolled_map[edge]) {
				int a1 = EDGE_TO_A1(prob, edge);
				cut_lhs.setLinearCoef(t[a1], -1);

				xk[a1] = 1;
				objk += tvals[a1];
			}
			else {
				int a2 = EDGE_TO_A2(prob, edge);
				yk[a2] = 1;
			}

			current = prev;
		}

		cuts.add(cut_lhs <= cut_rhs);

		xvals.add(xk);
		yvals.add(yk);
		obj += objk * prob.commodities[k].demand;
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
	ss << "OBJ: " << cplex.getObjValue() << endl <<
		"TIME: " << getTime() << " s" <<
		"    Sep " << separate_time << " s" <<
		"    Avg " << (separate_time * 1000 / separate_count) << " ms" <<
		"    Sub " << (subprob_time * 100 / separate_time) << "%" << endl <<
		"SEP: Total " << separate_count << endl;

	return ss.str();
}

vector<pair<IloCplex::Callback::Function*, value_func_model::ContextId>> value_func_model::attach_callbacks()
{
	return {
		make_pair(new value_func_model_callback(*this), CPX_CALLBACKCONTEXT_CANDIDATE | CPX_CALLBACKCONTEXT_RELAXATION)
	};
}
