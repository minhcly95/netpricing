#include "value_func_formulation.h"

#include "../base/hybrid_model.h"
#include "../base/processed_var_name.h"
#include "../../utilities/cplex_compare.h"

using namespace std;

value_func_formulation::value_func_formulation() :
	processed_formulation()
{
}

value_func_formulation::value_func_formulation(preprocessor* _preproc) :
	processed_formulation(_preproc)
{
}

value_func_formulation::~value_func_formulation()
{
}

IloRange value_func_formulation::get_cut(const std::vector<int>& path)
{
	cost_type path_cost = lgraph->get_path_cost(path, false);
	light_graph::toll_set tset = lgraph->get_toll_set(path);

	// Cut formulation
	IloExpr cut_lhs(env);
	IloNum cut_rhs = 0;

	// Fixed part
	LOOP_INFO(a, A1) {
		cost_type cost = info.cost_A1.at(a);
		cut_lhs.setLinearCoef(x[a], cost);
		cut_lhs.setLinearCoef(tx[a], 1);
	}
	LOOP_INFO(a, A2) {
		cost_type cost = info.cost_A2.at(a);
		cut_lhs.setLinearCoef(y[a], cost);
	}

	// Path-depending part
	cut_rhs = path_cost;
	for (const auto& pair : tset) {
		int a1 = info.bimap_A1.right.at(pair);
		cut_lhs.setLinearCoef(t[a1], -1);
	}

	return cut_lhs <= cut_rhs * (1 + TOLERANCE);
}

void value_func_formulation::formulate_impl()
{
	preprocess();

	// Variables
	x = VarArray(env, A1, 0, 1, ILOBOOL);
	y = VarArray(env, A2, 0, IloInfinity);
	tx = VarArray(env, A1, 0, IloInfinity);

	SET_VAR_NAMES_K(info, k, x, y, tx);

	LOOP_INFO(a, A1) cplex_model.add(x[a]);
	LOOP_INFO(a, A2) cplex_model.add(y[a]);
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
	LOOP_INFO(a, A1) cplex_model.add(bilinear1[a]);
	LOOP_INFO(a, A1) cplex_model.add(bilinear2[a]);
	LOOP_INFO(a, A1) cplex_model.add(bilinear3[a]);
}

std::vector<IloNumVar> value_func_formulation::get_all_variables()
{
	std::vector<IloNumVar> vars;

	LOOP_INFO(a, A1) vars.push_back(x[a]);
	LOOP_INFO(a, A2) vars.push_back(y[a]);
	LOOP_INFO(a, A1) vars.push_back(tx[a]);

	return vars;
}

IloExpr value_func_formulation::get_obj_expr()
{
	IloExpr expr(env);
	LOOP_INFO(a, A1) expr.setLinearCoef(tx[a], prob->commodities[k].demand);
	return expr;
}

std::vector<std::pair<IloNumVar, IloNum>> value_func_formulation::path_to_solution(const NumArray& tvals, const std::vector<int>& _)
{
	map<IloNumVar, IloNum> sol;

	LOOP_INFO(a, A1) sol.emplace(x[a], 0);
	LOOP_INFO(a, A2) sol.emplace(y[a], 0);
	LOOP_INFO(a, A1) sol.emplace(tx[a], 0);

	// Resolve the path for the processed graph
	vector<int> path = get_path(tvals);
	
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

	return vector<pair<IloNumVar, IloNum>>(sol.begin(), sol.end());
}

void value_func_formulation::invoke_callback(const IloCplex::Callback::Context& context, const NumArray& tvals)
{
	// Solve
	cb_path = get_path(tvals);

	// Cut formulation
	IloRange cut = get_cut(cb_path);

	// Add the cuts if it is violated
	double lhs_val = context.getCandidateValue(cut.getExpr());
	if (lhs_val > cut.getUB()) {
		context.rejectCandidate(cut);
	}

	// Clean up
	cut.end();
}

vector<int> value_func_formulation::get_callback_optimal_path()
{
	return cb_path;
}

double value_func_formulation::get_callback_obj()
{
	// Undo toll preference multiplier
	return lgraph->get_path_toll(cb_path) * prob->commodities[k].demand / TOLL_PREFERENCE;
}

void value_func_formulation::post_heuristic_cut(const IloCplex::Callback::Context& context, const NumArray& tvals, const std::vector<int>& _)
{
	// We don't use the given path
	// Resolve the path for the processed graph
	IloRange cut = get_cut(get_path(tvals));
	context.addUserCut(cut, IloCplex::CutManagement::UseCutFilter, false);
	cut.end();
}
