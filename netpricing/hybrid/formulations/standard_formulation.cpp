#include "standard_formulation.h"

#include "../base/hybrid_model.h"
#include "../../macros.h"
#include "../../utilities/set_var_name.h"
#include "../../utilities/cplex_compare.h"
#include "../../graph/light_graph.h"

using namespace std;

standard_formulation::standard_formulation() :
	processed_formulation()
{
}

standard_formulation::standard_formulation(const std::vector<path>& paths) :
	processed_formulation(paths)
{
}

standard_formulation::~standard_formulation()
{
	delete lgraph;
}

void standard_formulation::formulate_impl()
{
	process_graph();

	lgraph = new light_graph(prob->graph);

	// Variables
	x = VarArray(env, A1, 0, 1, ILOBOOL);
	y = VarArray(env, A2, 0, IloInfinity);
	lambda = VarArray(env, V, -IloInfinity, IloInfinity);
	tx = VarArray(env, A1, 0, IloInfinity);

	SET_VAR_NAMES_K(model, k, x, y, tx, lambda);

	LOOP_VALID(a, A1) cplex_model.add(x[a]);
	LOOP_VALID(a, A2) cplex_model.add(y[a]);
	LOOP_VALID(i, V) cplex_model.add(lambda[i]);
	LOOP_VALID(a, A1) cplex_model.add(tx[a]);

	// Objective
	LOOP_VALID(a, A1) obj.setLinearCoef(tx[a], prob->commodities[k].demand);

	// Flow constraints
	flow_constr = RangeArray(env, V, 0, 0);
	flow_constr[prob->commodities[k].origin].setBounds(1, 1);
	flow_constr[prob->commodities[k].destination].setBounds(-1, -1);

	// Flow matrix
	LOOP_VALID(a, A1) {
		SRC_DST_FROM_A1(*prob, a);
		flow_constr[src].setLinearCoef(x[a], 1);
		flow_constr[dst].setLinearCoef(x[a], -1);
	}
	LOOP_VALID(a, A2) {
		SRC_DST_FROM_A2(*prob, a);
		flow_constr[src].setLinearCoef(y[a], 1);
		flow_constr[dst].setLinearCoef(y[a], -1);
	}

	// Dual feasibility
	dual_feas = RangeArray(env, A);
	LOOP_VALID(a, A) {
		SRC_DST_FROM_A(*prob, a);
		bool is_tolled = prob->is_tolled_map[edge];
		cost_type cost = prob->cost_map[edge];

		dual_feas[a] = IloRange(lambda[src] - lambda[dst] <= cost);
		if (is_tolled)
			dual_feas[a].setLinearCoef(t[A_TO_A1(*prob, a)], -1);
	}

	// Equal objective
	equal_obj = IloRange(env, 0, 0);
	LOOP_VALID(a, A1) {
		auto edge = A1_TO_EDGE(*prob, a);
		cost_type cost = prob->cost_map[edge];
		equal_obj.setLinearCoef(x[a], cost);
		equal_obj.setLinearCoef(tx[a], 1);
	}
	LOOP_VALID(a, A2) {
		auto edge = A2_TO_EDGE(*prob, a);
		cost_type cost = prob->cost_map[edge];
		equal_obj.setLinearCoef(y[a], cost);
	}
	equal_obj.setLinearCoef(lambda[prob->commodities[k].origin], -1);
	equal_obj.setLinearCoef(lambda[prob->commodities[k].destination], 1);

	// Bilinear 1
	bilinear1 = RangeArray(env, A1, -IloInfinity, 0);
	LOOP_VALID(a, A1) {
		bilinear1[a].setLinearCoef(tx[a], 1);
		bilinear1[a].setLinearCoef(x[a], -prob->big_m[k][a]);
	}

	// Bilinear 2
	bilinear2 = RangeArray(env, A1);
	LOOP_VALID(a, A1) {
		bilinear2[a] = IloRange(env, -IloInfinity, prob->big_n[a]);
		bilinear2[a].setLinearCoef(t[a], 1);
		bilinear2[a].setLinearCoef(tx[a], -1);
		bilinear2[a].setLinearCoef(x[a], prob->big_n[a]);
	}

	// Bilinear 3
	bilinear3 = RangeArray(env, A1, -IloInfinity, 0);
	LOOP_VALID(a, A1) {
		bilinear3[a].setLinearCoef(t[a], -1);
		bilinear3[a].setLinearCoef(tx[a], 1);
	}

	// Add to model
	LOOP_VALID(i, V) cplex_model.add(flow_constr[i]);
	LOOP_VALID(a, A) cplex_model.add(dual_feas[a]);
	cplex_model.add(equal_obj);
	LOOP_VALID(a, A1) cplex_model.add(bilinear1[a]);
	LOOP_VALID(a, A1) cplex_model.add(bilinear2[a]);
	LOOP_VALID(a, A1) cplex_model.add(bilinear3[a]);
}

std::vector<IloNumVar> standard_formulation::get_all_variables()
{
	std::vector<IloNumVar> vars;

	LOOP_VALID(a, A1) vars.push_back(x[a]);
	LOOP_VALID(a, A2) vars.push_back(y[a]);
	LOOP_VALID(i, V) vars.push_back(lambda[i]);
	LOOP_VALID(a, A1) vars.push_back(tx[a]);

	return vars;
}

IloExpr standard_formulation::get_obj_expr()
{
	IloExpr expr(env);
	LOOP_VALID(a, A1) expr.setLinearCoef(tx[a], prob->commodities[k].demand);
	return expr;
}

std::vector<std::pair<IloNumVar, IloNum>> standard_formulation::path_to_solution(const NumArray& tvals, const std::vector<int>& path)
{
	map<IloNumVar, IloNum> sol;

	LOOP_VALID(a, A1) sol.emplace(x[a], 0);
	LOOP_VALID(a, A2) sol.emplace(y[a], 0);
	LOOP_VALID(i, V) sol.emplace(lambda[i], 0);
	LOOP_VALID(a, A1) sol.emplace(tx[a], 0);

	// Set the variables of the edges in path to 1
	for (int i = 0; i < path.size() - 1; i++) {
		auto edge = EDGE_FROM_SRC_DST(*prob, path[i], path[i + 1]);
		bool is_tolled = prob->is_tolled_map[edge];
		if (is_tolled) {
			int a1 = EDGE_TO_A1(*prob, edge);
			sol.at(x[a1]) = 1;
			sol.at(tx[a1]) = tvals[a1];
		}
		else
			sol.at(y[EDGE_TO_A2(*prob, edge)]) = 1;
	}

	// Set toll
	LOOP(a, A1) {
		SRC_DST_FROM_A1(*prob, a);
		lgraph->edge(src, dst).toll = tvals[a];
	}

	// Set lambda to the price to dst
	auto prices = lgraph->price_to_dst(prob->commodities[k].destination);
	LOOP_VALID(i, V) sol.at(lambda[i]) = prices[i];

	return vector<pair<IloNumVar, IloNum>>(sol.begin(), sol.end());
}