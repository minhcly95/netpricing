#include "path_formulation.h"

#include "../base/hybrid_model.h"
#include "../../macros.h"
#include "../../utilities/set_var_name.h"
#include "../../utilities/cplex_compare.h"
#include "../../graph/light_graph.h"

using namespace std;

path_formulation::path_formulation(const std::vector<path>& paths, bool full_mode) :
	path_based_formulation(paths), full_mode(full_mode)
{
}

path_formulation::~path_formulation()
{
}

IloRange path_formulation::get_cut(int curr_p, int opt_p)
{
	// Cut formulation
	IloExpr cut_lhs(env);

	// Coef of z[p] = cost curr - cost opt
	cut_lhs.setLinearCoef(z[curr_p], null_costs[curr_p] - null_costs[opt_p]);

	// Coef of tz[p][a] = 1 if in curr, -1 if in opt, 0 if in both
	vector<int> a_pos, a_neg;

	set_difference(toll_sets[curr_p].begin(), toll_sets[curr_p].end(),
				   toll_sets[opt_p].begin(), toll_sets[opt_p].end(),
				   back_inserter(a_pos));

	set_difference(toll_sets[opt_p].begin(), toll_sets[opt_p].end(),
				   toll_sets[curr_p].begin(), toll_sets[curr_p].end(),
				   back_inserter(a_neg));

	for (int a : a_pos) cut_lhs.setLinearCoef(tz[curr_p][a], 1);
	for (int a : a_neg) cut_lhs.setLinearCoef(tz[curr_p][a], -1);

	return cut_lhs <= 0;
}

void path_formulation::formulate_impl()
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

	// Toll polyhedra (if full mode is on)
	if (full_mode) {
		polyhedra = RangeMatrix(env, P);
		LOOP(p, P) {
			polyhedra[p] = RangeArray(env, P);
			LOOP(q, P) {
				if (p == q) continue;
				polyhedra[p][q] = get_cut(p, q);
				cplex_model.add(polyhedra[p][q]);
			}
		}
	}
}

std::vector<IloNumVar> path_formulation::get_all_variables()
{
	std::vector<IloNumVar> vars;

	LOOP(p, P) vars.push_back(z[p]);
	LOOP(p, P) LOOP(a, A1) vars.push_back(tz[p][a]);

	return vars;
}

IloExpr path_formulation::get_obj_expr()
{
	IloExpr expr(env);
	LOOP(p, P) for (int a : toll_sets[p])
		expr.setLinearCoef(tz[p][a], prob->commodities[k].demand);
	return expr;
}

std::vector<std::pair<IloNumVar, IloNum>> path_formulation::path_to_solution(const NumArray& tvals, const std::vector<int>& path)
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

void path_formulation::invoke_callback(const IloCplex::Callback::Context& context, const NumArray& tvals)
{
	// Extract z
	IloNumArray zvals(env, P);
	context.getCandidatePoint(z, zvals);

	// Find the current path
	int current_p = -1;
	LOOP(p, P) if (zvals[p] > 0.5) {
		current_p = p;
		break;
	}
	assert(current_p >= 0);

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

	if (current_p != cb_opt_p) {
		// Cut formulation
		IloRange cut = get_cut(current_p, cb_opt_p);

		// Add the cuts if it is violated
		double lhs_val = context.getCandidateValue(cut.getExpr());
		if (lhs_val > TOLERANCE) {
			context.rejectCandidate(cut);
		}

		cut.end();
	}

	// Clean up
	zvals.end();
}

vector<int> path_formulation::get_callback_optimal_path()
{
	return paths[cb_opt_p];
}

double path_formulation::get_callback_obj()
{
	return cb_opt_toll * prob->commodities[k].demand;
}
