#include "arc_path_value_func_formulation.h"

#include "../base/hybrid_model.h"
#include "../../macros.h"
#include "../../utilities/set_var_name.h"
#include "../base/processed_var_name.h"
#include "../../utilities/cplex_compare.h"
#include "../../graph/light_graph.h"

using namespace std;

arc_path_value_func_formulation::arc_path_value_func_formulation(const std::vector<path>& paths) :
	path_based_formulation(paths), preproc(paths)
{
}

arc_path_value_func_formulation::~arc_path_value_func_formulation()
{
}

void arc_path_value_func_formulation::formulate_impl()
{
	// Prepare the solver
	prepare();
	info = preproc.preprocess(*prob, k);
	V = *(info.V.rbegin()) + 1;
	A = *(info.A.rbegin()) + 1;
	A1 = *(info.A1.rbegin()) + 1;
	A2 = *(info.A2.rbegin()) + 1;
	lgraph = new light_graph(info.build_graph());

	// Variables
	z = VarArray(env, P, 0, 1, ILOBOOL);
	tx = VarArray(env, A1, 0, IloInfinity);

	SET_VAR_NAMES_K(model, k, z);
	SET_VAR_NAMES_K(info, k, tx);

	cplex_model.add(z);
	LOOP_INFO(a, A1) cplex_model.add(tx[a]);

	// Objective
	LOOP_INFO(a, A1) obj.setLinearCoef(tx[a], prob->commodities[k].demand);

	// Sum z = 1
	sum_z = IloRange(env, 1, 1);
	LOOP(p, P) sum_z.setLinearCoef(z[p], 1);

	// Value function constraints
	valuefunc = RangeArray(env, P, -IloInfinity, 0);
	LOOP(q, P) {
		// Fixed part
		LOOP(p, P) valuefunc[q].setLinearCoef(z[p], null_costs[p]);
		LOOP_INFO(a, A1) valuefunc[q].setLinearCoef(tx[a], 1);
		// Path-depending part
		valuefunc[q].setUB(null_costs[q]);
		for (int a : toll_sets[q]) valuefunc[q].setLinearCoef(t[a], -1);
	}

	// Bilinear 1
	bilinear1 = RangeArray(env, A1, -IloInfinity, 0);
	LOOP_INFO(a, A1) {
		bilinear1[a].setLinearCoef(tx[a], 1);
		LOOP(p, P) if (toll_sets[p].count(a))
			bilinear1[a].setLinearCoef(z[p], -prob->big_m[k][a]);
	}

	// Bilinear 2
	bilinear2 = RangeArray(env, A1);
	LOOP_INFO(a, A1) {
		bilinear2[a] = IloRange(env, -IloInfinity, prob->big_n[a]);
		bilinear2[a].setLinearCoef(t[a], 1);
		bilinear2[a].setLinearCoef(tx[a], -1);
		LOOP(p, P) if (toll_sets[p].count(a))
			bilinear2[a].setLinearCoef(z[p], prob->big_n[a]);
	}

	// Bilinear 3
	bilinear3 = RangeArray(env, A1, -IloInfinity, 0);
	LOOP_INFO(a, A1) {
		bilinear3[a].setLinearCoef(t[a], -1);
		bilinear3[a].setLinearCoef(tx[a], 1);
	}

	// Add to model
	cplex_model.add(sum_z);
	cplex_model.add(valuefunc);
	LOOP_INFO(a, A1) cplex_model.add(bilinear1[a]);
	LOOP_INFO(a, A1) cplex_model.add(bilinear2[a]);
	LOOP_INFO(a, A1) cplex_model.add(bilinear3[a]);
}

std::vector<IloNumVar> arc_path_value_func_formulation::get_all_variables()
{
	std::vector<IloNumVar> vars;

	LOOP(p, P) vars.push_back(z[p]);
	LOOP_INFO(a, A1) vars.push_back(tx[a]);

	return vars;
}

IloExpr arc_path_value_func_formulation::get_obj_expr()
{
	IloExpr expr(env);
	LOOP_INFO(a, A1) expr.setLinearCoef(tx[a], prob->commodities[k].demand);
	return expr;
}

std::vector<std::pair<IloNumVar, IloNum>> arc_path_value_func_formulation::path_to_solution(const NumArray& tvals, const std::vector<int>& path)
{
	map<IloNumVar, IloNum> sol;

	LOOP(p, P) sol.emplace(z[p], 0);
	LOOP_INFO(a, A1) sol.emplace(tx[a], 0);

	// Get the index of the given path
	auto it = std::find(paths.begin(), paths.end(), path);
	assert(it != paths.end());		// Must be a bilevel feasible path
	int p_given = std::distance(paths.begin(), it);

	sol.at(z[p_given]) = 1;

	// Set tx along the path
	for (int a : toll_sets[p_given])
		sol.at(tx[a]) = tvals[a];

	return vector<pair<IloNumVar, IloNum>>(sol.begin(), sol.end());
}