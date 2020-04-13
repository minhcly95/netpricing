#include "value_func_formulation.h"

#include "../base/hybrid_model.h"
#include "../../macros.h"
#include "../../utilities/set_var_name.h"
#include "../../utilities/cplex_compare.h"
#include "../../graph/light_graph.h"

using namespace std;

value_func_formulation::~value_func_formulation()
{
	delete lgraph;
}

void value_func_formulation::formulate_impl()
{
	// Prepare the solver
	lgraph = new light_graph(prob->graph);

	// Variables
	x = VarArray(env, A1, 0, 1, ILOBOOL);
	y = VarArray(env, A2, 0, IloInfinity);
	tx = VarArray(env, A1, 0, IloInfinity);

	SET_VAR_NAMES_K(model, k, x, y, tx);

	cplex_model.add(x);
	cplex_model.add(y);
	cplex_model.add(tx);

	// Objective
	LOOP(a, A1) obj.setLinearCoef(tx[a], prob->commodities[k].demand);

	// Flow constraints
	flow_constr = RangeArray(env, V, 0, 0);
	flow_constr[prob->commodities[k].origin].setBounds(1, 1);
	flow_constr[prob->commodities[k].destination].setBounds(-1, -1);

	// Flow matrix
	LOOP(a, A1) {
		SRC_DST_FROM_A1(*prob, a);
		flow_constr[src].setLinearCoef(x[a], 1);
		flow_constr[dst].setLinearCoef(x[a], -1);
	}
	LOOP(a, A2) {
		SRC_DST_FROM_A2(*prob, a);
		flow_constr[src].setLinearCoef(y[a], 1);
		flow_constr[dst].setLinearCoef(y[a], -1);
	}

	// Bilinear 1
	bilinear1 = RangeArray(env, A1, -IloInfinity, 0);
	LOOP(a, A1) {
		bilinear1[a].setLinearCoef(tx[a], 1);
		bilinear1[a].setLinearCoef(x[a], -prob->big_m[k][a]);
	}

	// Bilinear 2
	bilinear2 = RangeArray(env, A1);
	LOOP(a, A1) {
		bilinear2[a] = IloRange(env, -IloInfinity, prob->big_n[a]);
		bilinear2[a].setLinearCoef(t[a], 1);
		bilinear2[a].setLinearCoef(tx[a], -1);
		bilinear2[a].setLinearCoef(x[a], prob->big_n[a]);
	}

	// Bilinear 3
	bilinear3 = RangeArray(env, A1, -IloInfinity, 0);
	LOOP(a, A1) {
		bilinear3[a].setLinearCoef(t[a], -1);
		bilinear3[a].setLinearCoef(tx[a], 1);
	}

	// Add to model
	cplex_model.add(flow_constr);
	cplex_model.add(bilinear1);
	cplex_model.add(bilinear2);
	cplex_model.add(bilinear3);
}

void value_func_formulation::invoke_callback(const IloCplex::Callback::Context& context, const NumArray& tvals)
{
	// Set toll
	LOOP(a, A1) {
		SRC_DST_FROM_A1(*prob, a);
		lgraph->edge(src, dst).toll = tvals[a] * TOLL_PREFERANCE;		// Prefer tolled arcs
	}

	// Solve
	cb_path = lgraph->shortest_path(prob->commodities[k].origin, prob->commodities[k].destination);
	cost_type cost = lgraph->get_path_cost(cb_path, false);
	light_graph::toll_set tset = lgraph->get_toll_set(cb_path);

	// Cut formulation
	IloExpr cut_lhs(env);
	IloNum cut_rhs = 0;

	// Fixed part
	LOOP(a, A1) {
		auto edge = A1_TO_EDGE(*prob, a);
		cost_type cost = prob->cost_map[edge];
		cut_lhs.setLinearCoef(x[a], cost);
		cut_lhs.setLinearCoef(tx[a], 1);
	}
	LOOP(a, A2) {
		auto edge = A2_TO_EDGE(*prob, a);
		cost_type cost = prob->cost_map[edge];
		cut_lhs.setLinearCoef(y[a], cost);
	}

	// Path-depending part
	cut_rhs = cost;
	for (const auto& pair : tset) {
		auto edge = EDGE_FROM_SRC_DST(*prob, pair.first, pair.second);
		int a1 = EDGE_TO_A1(*prob, edge);
		cut_lhs.setLinearCoef(t[a1], -1);
	}

	// Add the cuts if it is violated
	double lhs_val = context.getCandidateValue(cut_lhs);
	if (lhs_val > cut_rhs* (1 + TOLERANCE)) {
		context.rejectCandidate(cut_lhs <= cut_rhs);
	}

	// Clean up
	cut_lhs.end();
}

vector<pair<IloNumVar, IloNum>> value_func_formulation::get_callback_solution(const NumArray& tvals)
{
	map<IloNumVar, IloNum> sol;

	LOOP(a, A1) sol.emplace(x[a], 0);
	LOOP(a, A2) sol.emplace(y[a], 0);
	LOOP(a, A1) sol.emplace(tx[a], 0);

	// Set the variables of the edges in cb_path to 1
	for (int i = 0; i < cb_path.size() - 1; i++) {
		auto edge = EDGE_FROM_SRC_DST(*prob, cb_path[i], cb_path[i + 1]);
		bool is_tolled = prob->is_tolled_map[edge];
		if (is_tolled) {
			int a1 = EDGE_TO_A1(*prob, edge);
			sol.at(x[a1]) = 1;
			sol.at(tx[a1]) = tvals[a1];
		}
		else
			sol.at(y[EDGE_TO_A2(*prob, edge)]) = 1;
	}

	return vector<pair<IloNumVar, IloNum>>(sol.begin(), sol.end());
}

double value_func_formulation::get_callback_obj()
{
	// Undo toll preferance multiplier
	return lgraph->get_path_toll(cb_path) * prob->commodities[k].demand / TOLL_PREFERANCE;
}
