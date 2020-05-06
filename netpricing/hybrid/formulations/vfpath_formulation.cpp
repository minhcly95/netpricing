#include "vfpath_formulation.h"

#include "../base/hybrid_model.h"
#include "../../macros.h"
#include "../../utilities/set_var_name.h"
#include "../../utilities/cplex_compare.h"
#include "../../graph/light_graph.h"

using namespace std;

vfpath_formulation::vfpath_formulation(const std::vector<path>& paths, bool full_mode) :
	paths(paths), P(paths.size()), full_mode(full_mode)
{
}

vfpath_formulation::~vfpath_formulation()
{
}

void vfpath_formulation::prepare()
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

IloRange vfpath_formulation::get_cut(int target_p)
{
	// Cut formulation
	IloExpr cut_lhs(env);
	IloNum cut_rhs;

	// Fixed part
	LOOP(p, P) {
		cut_lhs.setLinearCoef(z[p], null_costs[p]);
		for (int a : toll_sets[p])
			cut_lhs.setLinearCoef(tz[p][a], 1);
	}

	// Path-depending part
	cut_rhs = null_costs[target_p];
	for (int a : toll_sets[target_p])
		cut_lhs.setLinearCoef(t[a], -1);

	return cut_lhs <= cut_rhs;
}

void vfpath_formulation::formulate_impl()
{
	// Prepare the solver
	prepare();

	// Variables
	z = VarArray(env, P, 0, 1, ILOBOOL);
	tz = VarMatrix(env, P);
	LOOP(p, P) tz[p] = VarArray(env, A1, 0, IloInfinity);

	SET_VAR_NAMES_K(model, k, z, tz);

	cplex_model.add(z);
	LOOP(p, P) cplex_model.add(tz[p]);

	// Objective
	LOOP(p, P) for (int a : toll_sets[p])
		obj.setLinearCoef(tz[p][a], prob->commodities[k].demand);

	// Sum z = 1
	sum_z = IloRange(env, 1, 1);
	LOOP(p, P) sum_z.setLinearCoef(z[p], 1);

	// Sum tz = t
	sum_tz = RangeArray(env, A1, 0, 0);
	LOOP(a, A1) {
		sum_tz[a].setLinearCoef(t[a], -1);
		LOOP(p, P) sum_tz[a].setLinearCoef(tz[p][a], 1);
	}

	// Upper bound for tz
	upperbound = RangeMatrix(env, P);
	LOOP(p, P) {
		upperbound[p] = RangeArray(env, A1, -IloInfinity, 0);
		LOOP(a, A1) {
			upperbound[p][a].setLinearCoef(tz[p][a], 1);
			upperbound[p][a].setLinearCoef(z[p], -prob->big_n[a]);
		}
	}

	// Add to model
	cplex_model.add(sum_z);
	cplex_model.add(sum_tz);
	LOOP(p, P) cplex_model.add(upperbound[p]);

	// Value function constraints (if full mode is on)
	if (full_mode) {
		valuefunc = RangeArray(env, P);
		LOOP(p, P) {
			valuefunc[p] = get_cut(p);
			cplex_model.add(valuefunc[p]);
		}
	}
}

std::vector<IloNumVar> vfpath_formulation::get_all_variables()
{
	std::vector<IloNumVar> vars;

	LOOP(p, P) vars.push_back(z[p]);
	LOOP(p, P) LOOP(a, A1) vars.push_back(tz[p][a]);

	return vars;
}

IloExpr vfpath_formulation::get_obj_expr()
{
	IloExpr expr(env);
	LOOP(p, P) for (int a : toll_sets[p])
		expr.setLinearCoef(tz[p][a], prob->commodities[k].demand);
	return expr;
}

std::vector<std::pair<IloNumVar, IloNum>> vfpath_formulation::path_to_solution(const NumArray& tvals, const std::vector<int>& path)
{
	// Get the index of the given path
	auto it = std::find(paths.begin(), paths.end(), path);
	assert(it != paths.end());		// Must be a bilevel feasible path
	int p_given = std::distance(paths.begin(), it);

	vector<pair<IloNumVar, IloNum>> sol;

	LOOP(p, P) {
		if (p != p_given) {
			sol.emplace_back(z[p], 0);
			LOOP(a, A1) sol.emplace_back(tz[p][a], 0);
		}
		else {
			sol.emplace_back(z[p], 1);
			LOOP(a, A1) sol.emplace_back(tz[p][a], tvals[a]);
		}
	}

	return sol;
}

void vfpath_formulation::invoke_callback(const IloCplex::Callback::Context& context, const NumArray& tvals)
{
	// Find the best path
	vector<cost_type> costs = null_costs;
	LOOP(p, P) for (int a : toll_sets[p])
		costs[p] += tvals[a] * TOLL_PREFERENCE;

	auto it = std::min_element(costs.begin(), costs.end());
	cb_opt_p = std::distance(costs.begin(), it);	// Callback optimal path index

	// Optimal objective
	cb_opt_toll = 0;
	for (int a : toll_sets[cb_opt_p])
		cb_opt_toll += tvals[a];

	// Cut formulation
	IloRange cut = get_cut(cb_opt_p);

	// Add the cuts if it is violated
	double lhs_val = context.getCandidateValue(cut.getExpr());
	if (lhs_val > cut.getUB() * (1 + TOLERANCE)) {
		context.rejectCandidate(cut);
	}

	// Clean up
	cut.end();
}

vector<int> vfpath_formulation::get_callback_optimal_path()
{
	return paths[cb_opt_p];
}

double vfpath_formulation::get_callback_obj()
{
	return cb_opt_toll * prob->commodities[k].demand;
}

void vfpath_formulation::post_heuristic_cut(const IloCplex::Callback::Context& context, const NumArray& tvals, const std::vector<int>& path)
{
	// Get the index of the given path
	auto it = std::find(paths.begin(), paths.end(), path);
	assert(it != paths.end());		// Must be a bilevel feasible path
	int p_given = std::distance(paths.begin(), it);

	// Add the cut
	IloRange cut = get_cut(p_given);
	context.addUserCut(cut, IloCplex::CutManagement::UseCutFilter, false);
	cut.end();
}
