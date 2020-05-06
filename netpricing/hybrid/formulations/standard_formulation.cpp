#include "standard_formulation.h"

#include "../base/hybrid_model.h"
#include "../base/processed_var_name.h"
#include "../../utilities/cplex_compare.h"

using namespace std;

standard_formulation::standard_formulation() :
	processed_formulation()
{
}

standard_formulation::standard_formulation(preprocessor* _preproc) :
	processed_formulation(_preproc)
{
}

standard_formulation::~standard_formulation()
{
}

void standard_formulation::formulate_impl()
{
	preprocess();

	// Variables
	x = VarArray(env, A1, 0, 1, ILOBOOL);
	y = VarArray(env, A2, 0, IloInfinity);
	lambda = VarArray(env, V, -IloInfinity, IloInfinity);
	tx = VarArray(env, A1, 0, IloInfinity);

	SET_VAR_NAMES_K(info, k, x, y, tx, lambda);

	LOOP_INFO(a, A1) cplex_model.add(x[a]);
	LOOP_INFO(a, A2) cplex_model.add(y[a]);
	LOOP_INFO(i, V) cplex_model.add(lambda[i]);
	LOOP_INFO(a, A1) cplex_model.add(tx[a]);

	// Objective
	LOOP_INFO(a, A1) obj.setLinearCoef(tx[a], prob->commodities[k].demand);

	// Flow constraints
	flow_constr = RangeArray(env, V, 0, 0);
	flow_constr[prob->commodities[k].origin].setBounds(1, 1);
	flow_constr[prob->commodities[k].destination].setBounds(-1, -1);

	// Flow matrix
	LOOP_INFO(a, A1) {
		auto arc = info.bimap_A1.left.at(a);
		flow_constr[arc.first].setLinearCoef(x[a], 1);
		flow_constr[arc.second].setLinearCoef(x[a], -1);
	}
	LOOP_INFO(a, A2) {
		auto arc = info.bimap_A2.left.at(a);
		flow_constr[arc.first].setLinearCoef(y[a], 1);
		flow_constr[arc.second].setLinearCoef(y[a], -1);
	}

	// Dual feasibility
	dual_feas = RangeArray(env, A);
	LOOP_INFO(a, A) {
		auto arc = info.bimap_A.left.at(a);
		bool is_tolled = info.is_tolled.at(a);
		cost_type cost = info.cost_A.at(a);

		dual_feas[a] = IloRange(lambda[arc.first] - lambda[arc.second] <= cost);
		if (is_tolled)
			dual_feas[a].setLinearCoef(t[info.a_to_a1(a)], -1);
	}

	// Equal objective
	equal_obj = IloRange(env, 0, 0);
	LOOP_INFO(a, A1) {
		cost_type cost = info.cost_A1.at(a);
		equal_obj.setLinearCoef(x[a], cost);
		equal_obj.setLinearCoef(tx[a], 1);
	}
	LOOP_INFO(a, A2) {
		cost_type cost = info.cost_A2.at(a);
		equal_obj.setLinearCoef(y[a], cost);
	}
	equal_obj.setLinearCoef(lambda[prob->commodities[k].origin], -1);
	equal_obj.setLinearCoef(lambda[prob->commodities[k].destination], 1);

	// Bilinear 1
	bilinear1 = RangeArray(env, A1, -IloInfinity, 0);
	LOOP_INFO(a, A1) {
		bilinear1[a].setLinearCoef(tx[a], 1);
		bilinear1[a].setLinearCoef(x[a], -prob->big_m[k][a]);
	}

	// Bilinear 2
	bilinear2 = RangeArray(env, A1);
	LOOP_INFO(a, A1) {
		bilinear2[a] = IloRange(env, -IloInfinity, prob->big_n[a]);
		bilinear2[a].setLinearCoef(t[a], 1);
		bilinear2[a].setLinearCoef(tx[a], -1);
		bilinear2[a].setLinearCoef(x[a], prob->big_n[a]);
	}

	// Bilinear 3
	bilinear3 = RangeArray(env, A1, -IloInfinity, 0);
	LOOP_INFO(a, A1) {
		bilinear3[a].setLinearCoef(t[a], -1);
		bilinear3[a].setLinearCoef(tx[a], 1);
	}

	// Add to model
	LOOP_INFO(i, V) cplex_model.add(flow_constr[i]);
	LOOP_INFO(a, A) cplex_model.add(dual_feas[a]);
	cplex_model.add(equal_obj);
	LOOP_INFO(a, A1) cplex_model.add(bilinear1[a]);
	LOOP_INFO(a, A1) cplex_model.add(bilinear2[a]);
	LOOP_INFO(a, A1) cplex_model.add(bilinear3[a]);
}

std::vector<IloNumVar> standard_formulation::get_all_variables()
{
	std::vector<IloNumVar> vars;

	LOOP_INFO(a, A1) vars.push_back(x[a]);
	LOOP_INFO(a, A2) vars.push_back(y[a]);
	LOOP_INFO(i, V) vars.push_back(lambda[i]);
	LOOP_INFO(a, A1) vars.push_back(tx[a]);

	return vars;
}

IloExpr standard_formulation::get_obj_expr()
{
	IloExpr expr(env);
	LOOP_INFO(a, A1) expr.setLinearCoef(tx[a], prob->commodities[k].demand);
	return expr;
}

std::vector<std::pair<IloNumVar, IloNum>> standard_formulation::path_to_solution(const NumArray& tvals, const std::vector<int>& _)
{
	map<IloNumVar, IloNum> sol;

	LOOP_INFO(a, A1) sol.emplace(x[a], 0);
	LOOP_INFO(a, A2) sol.emplace(y[a], 0);
	LOOP_INFO(i, V) sol.emplace(lambda[i], 0);
	LOOP_INFO(a, A1) sol.emplace(tx[a], 0);

	// Solve for path in processed graph
	auto path = get_path(tvals);

	// Set the variables of the edges in path to 1
	for (int i = 0; i < path.size() - 1; i++) {
		auto arc = make_pair(path[i], path[i + 1]);
		int a = info.bimap_A.right.at(arc);
		bool is_tolled = info.is_tolled.at(a);

		if (is_tolled) {
			int a1 = info.a_to_a1(a);
			sol.at(x[a1]) = 1;
			sol.at(tx[a1]) = tvals[a1];
		}
		else
			sol.at(y[info.a_to_a2(a)]) = 1;
	}

	// Set toll to true values
	LOOP_INFO(a, A1) {
		auto arc = info.bimap_A1.left.at(a);
		lgraph->edge(arc).toll = tvals[a];
	}

	// Set lambda to the price to dst
	auto prices = lgraph->price_to_dst(prob->commodities[k].destination);
	LOOP_INFO(i, V) sol.at(lambda[i]) = prices[i];

	return vector<pair<IloNumVar, IloNum>>(sol.begin(), sol.end());
}