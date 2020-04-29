#include "path_didi_formulation.h"

#include "../base/hybrid_model.h"
#include "../../macros.h"
#include "../../utilities/set_var_name.h"
#include "../../utilities/cplex_compare.h"
#include "../../graph/light_graph.h"

using namespace std;

path_didi_formulation::path_didi_formulation(const std::vector<path>& paths, bool full_mode) :
	paths(paths), P(paths.size()), full_mode(full_mode)
{
}

path_didi_formulation::~path_didi_formulation()
{
}

void path_didi_formulation::prepare()
{
	light_graph lgraph(prob->graph);

	null_costs.resize(P);
	LOOP(p, P) null_costs[p] = lgraph.get_path_cost(paths[p], false);

	toll_sets.resize(P);
	LOOP(p, P) {
		auto toll_arcs = lgraph.get_toll_list(paths[p]);
		for (const auto& pair : toll_arcs) {
			auto edge = EDGE_FROM_SRC_DST(*prob, pair.first, pair.second);
			toll_sets[p].insert(EDGE_TO_A1(*prob, edge));
		}
	}
}

void path_didi_formulation::formulate_impl()
{
	// Prepare the solver
	prepare();

	// Variables
	z = VarArray(env, P, 0, 1, ILOBOOL);
	tk = IloNumVar(env);
	lk = IloNumVar(env);

	SET_VAR_NAMES_K(model, k, z, tk, lk);

	cplex_model.add(z);
	cplex_model.add(tk);
	cplex_model.add(lk);

	// Objective
	obj.setLinearCoef(tk, prob->commodities[k].demand);

	// Sum z = 1
	sum_z = IloRange(env, 1, 1);
	LOOP(p, P) sum_z.setLinearCoef(z[p], 1);

	// Upper bound of lk
	lk_upper = RangeArray(env, A1, -IloInfinity, 0);
	LOOP(p, P) {
		lk_upper[p].setUb(null_costs[p]);
		lk_upper[p].setLinearCoef(lk, 1);
		for (int a : toll_sets[p])
			lk_upper[p].setLinearCoef(t[a], -1);
	}

	// Lower bound of lk
	lk_lower = RangeArray(env, A1, 0, IloInfinity);
	LOOP(p, P) {
		double big_m = 1000;
		lk_lower[p].setLb(null_costs[p] - big_m);
		lk_lower[p].setLinearCoef(lk, 1);
		lk_lower[p].setLinearCoef(z[p], -big_m);
		for (int a : toll_sets[p])
			lk_lower[p].setLinearCoef(t[a], -1);
	}

	// Constraint fixing tk to lk
	tk_constr = IloRange(env, 0, 0);
	tk_constr.setLinearCoef(tk, 1);
	tk_constr.setLinearCoef(lk, -1);
	LOOP(p, P)
		tk_constr.setLinearCoef(z[p], null_costs[p]);

	// Add to model
	cplex_model.add(sum_z);
	cplex_model.add(lk_upper);
	cplex_model.add(lk_lower);
	cplex_model.add(tk_constr);
}

std::vector<IloNumVar> path_didi_formulation::get_all_variables()
{
	std::vector<IloNumVar> vars;

	LOOP(p, P) vars.push_back(z[p]);
	vars.push_back(tk);
	vars.push_back(lk);

	return vars;
}

IloExpr path_didi_formulation::get_obj_expr()
{
	IloExpr expr(env);
	expr.setLinearCoef(tk, prob->commodities[k].demand);
	return expr;
}

std::vector<std::pair<IloNumVar, IloNum>> path_didi_formulation::path_to_solution(const NumArray& tvals, const std::vector<int>& path)
{
	// Get the index of the given path
	auto it = std::find(paths.begin(), paths.end(), path);
	assert(it != paths.end());		// Must be a bilevel feasible path
	int p_given = std::distance(paths.begin(), it);

	vector<pair<IloNumVar, IloNum>> sol;

	LOOP(p, P) {
		if (p != p_given) {
			sol.emplace_back(z[p], 0);
		}
		else {
			sol.emplace_back(z[p], 1);
			int tk_val = 0;
			for (int a : toll_sets[p])
				tk_val += tvals[a];
			sol.emplace_back(tk, tk_val);
			sol.emplace_back(lk, tk_val + null_costs[p]);
		}
	}

	return sol;
}
