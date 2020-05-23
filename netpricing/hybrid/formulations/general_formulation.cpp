#include "general_formulation.h"

#include "../base/hybrid_model.h"
#include "../../macros.h"
#include "../../utilities/set_var_name.h"
#include "../base/processed_var_name.h"
#include "../../utilities/cplex_compare.h"
#include "../../graph/light_graph.h"

#include <tuple>

using namespace std;

general_formulation::general_formulation(const std::vector<path>& paths, space primal_space, space dual_space, opt_condition opt_cond) :
	paths(paths), P(paths.size()), preproc(paths),
	primal_space(primal_space), dual_space(dual_space), opt_cond(opt_cond)
{
}

general_formulation::general_formulation(const std::vector<path>& paths, const std::string& code) :
	paths(paths), P(paths.size()), preproc(paths)
{
	auto model = std::tie(primal_space, dual_space, opt_cond);

	if (code == "std")
		model = make_tuple(ARC, ARC, STRONG_DUAL);
	else if (code == "apstd")
		model = make_tuple(PATH, ARC, STRONG_DUAL);
	else if (code == "vf")
		model = make_tuple(ARC, PATH, STRONG_DUAL);
	else if (code == "apvf")
		model = make_tuple(PATH, PATH, STRONG_DUAL);
	else if (code == "cs")
		model = make_tuple(ARC, ARC, COMP_SLACK);
	else if (code == "apcs")
		model = make_tuple(PATH, ARC, COMP_SLACK);
	else if (code == "vfcs")
		model = make_tuple(ARC, PATH, COMP_SLACK);
	else if (code == "didi")
		model = make_tuple(PATH, PATH, COMP_SLACK);
	else
		throw invalid_argument("invalid formulation code: " + code);
}

general_formulation::~general_formulation()
{
	delete lgraph;
}

void general_formulation::prepare()
{
	// Preprocessing
	info = preproc.preprocess(*prob, k);
	V = *(info.V.rbegin()) + 1;
	A = *(info.A.rbegin()) + 1;
	A1 = *(info.A1.rbegin()) + 1;
	A2 = *(info.A2.rbegin()) + 1;
	lgraph = new light_graph(info.build_graph());

	// Process path
	light_graph original(prob->graph);

	toll_sets.resize(P);
	arc_sets.resize(P);
	lgraph->set_toll_arcs_enabled(false);

	LOOP(p, P) {
		auto toll_arcs = original.get_toll_list(paths[p]);
		set<int> toll_heads;
		toll_heads.insert(prob->commodities[k].destination);

		// Extract toll sets and toll heads
		for (const auto& pair : toll_arcs) {
			auto edge = EDGE_FROM_SRC_DST(*prob, pair.first, pair.second);
			toll_sets[p].insert(EDGE_TO_A1(*prob, edge));
			toll_heads.insert(pair.first);
		}

		// Transform path
		vector<int> new_path;
		vector<int>& old_path = paths[p];
		auto it = old_path.begin();

		while (it != old_path.end()) {
			int src = *it;
			it = find_if(it, old_path.end(),
						 [&](int i) {
							 return toll_heads.count(i);
						 });
			assert(it != old_path.end());
			int dst = *it;

			if (src == dst) {
				new_path.push_back(src);
			}
			else {
				auto segment = lgraph->shortest_path(src, dst);
				assert(segment.size());
				new_path.insert(new_path.end(), segment.begin(), segment.end());
			}

			it++;
		}

		paths[p] = new_path;

		// Extract arc sets
		for (int i = 0; i < paths[p].size() - 1; i++) {
			int a = info.bimap_A.right.at(make_pair(paths[p][i], paths[p][i + 1]));
			arc_sets[p].insert(a);
		}
	}

	lgraph->set_toll_arcs_enabled(true);

	null_costs.resize(P);
	LOOP(p, P) null_costs[p] = lgraph->get_path_cost(paths[p], false);

}

bool general_formulation::check_model(space primal, space dual, opt_condition condition)
{
	return primal_space == primal && dual_space == dual && opt_cond == condition;
}

void general_formulation::formulate_impl()
{
	// Prepare
	prepare();

	// VARIABLES
	// Primal Arc
	if (primal_space == ARC) {
		x = VarArray(env, A1, 0, 1, ILOBOOL);
		if (opt_cond == COMP_SLACK)
			y = VarArray(env, A2, 0, 1, ILOBOOL);
		else
			y = VarArray(env, A2, 0, IloInfinity);

		SET_VAR_NAMES_K(info, k, x, y);
		LOOP_INFO(a, A1) cplex_model.add(x[a]);
		LOOP_INFO(a, A2) cplex_model.add(y[a]);
	}
	// Primal Path
	else {
		z = VarArray(env, P, 0, 1, ILOBOOL);
		SET_VAR_NAMES_K(model, k, z);
		cplex_model.add(z);
	}

	// Dual Arc
	if (dual_space == ARC) {
		lambda = VarArray(env, V, -IloInfinity, IloInfinity);
		lambda[prob->commodities[k].destination].setBounds(0, 0);	// Anchor destination to 0
		SET_VAR_NAMES_K(info, k, lambda);
		LOOP_INFO(i, V) cplex_model.add(lambda[i]);
	}
	// Dual Path
	else {
		lk = IloNumVar(env, -IloInfinity, IloInfinity);
		SET_VAR_NAMES_K(model, k, lk);
		cplex_model.add(lk);
	}

	// Strong duality
	if (opt_cond == STRONG_DUAL) {
		tx = VarArray(env, A1, 0, IloInfinity);
		SET_VAR_NAMES_K(info, k, tx);
		LOOP_INFO(a, A1) cplex_model.add(tx[a]);
	}
	// Complementary slackness
	else {
		tk = IloNumVar(env, -IloInfinity, IloInfinity);
		SET_VAR_NAMES_K(model, k, tk);
		cplex_model.add(tk);
	}

	// OBJECTIVE
	// Strong duality
	if (opt_cond == STRONG_DUAL)
		LOOP_INFO(a, A1) obj.setLinearCoef(tx[a], prob->commodities[k].demand);
	// Complementary slackness
	else
		obj.setLinearCoef(tk, prob->commodities[k].demand);

	// PRIMAL FEASIBILITY
	// Primal Arc
	if (primal_space == ARC) {
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
		LOOP_INFO(i, V) cplex_model.add(flow_constr[i]);
	}
	// Primal Path
	else {
		sum_z = IloRange(env, 1, 1);
		LOOP(p, P) sum_z.setLinearCoef(z[p], 1);
		cplex_model.add(sum_z);
	}

	// DUAL FEASIBILITY
	// Dual Arc
	if (dual_space == ARC) {
		dual_feas = RangeArray(env, A, -IloInfinity, 0);
		LOOP_INFO(a, A) {
			auto arc = info.bimap_A.left.at(a);
			bool is_tolled = info.is_tolled.at(a);
			cost_type cost = info.cost_A.at(a);

			dual_feas[a].setLinearCoef(lambda[arc.first], 1);
			dual_feas[a].setLinearCoef(lambda[arc.second], -1);
			dual_feas[a].setUB(cost);
			if (is_tolled)
				dual_feas[a].setLinearCoef(t[info.a_to_a1(a)], -1);
		}
		LOOP_INFO(a, A) cplex_model.add(dual_feas[a]);
	}
	// Dual Path
	else {
		valuefunc = RangeArray(env, P, -IloInfinity, 0);
		LOOP(p, P) {
			valuefunc[p].setLinearCoef(lk, 1);
			valuefunc[p].setUB(null_costs[p]);
			for (int a : toll_sets[p]) valuefunc[p].setLinearCoef(t[a], -1);
		}
		cplex_model.add(valuefunc);
	}

	// STRONG DUALITY (used for both conditions)
	if (opt_cond == STRONG_DUAL) {
		strong_dual = IloRange(env, 0, TOLERANCE);
		LOOP_INFO(a, A1) strong_dual.setLinearCoef(tx[a], 1);
	}
	else {
		strong_dual = IloRange(env, 0, 0);
		strong_dual.setLinearCoef(tk, 1);
	}

	// Primal
	if (primal_space == ARC) {
		LOOP_INFO(a, A1) strong_dual.setLinearCoef(x[a], info.cost_A1.at(a));
		LOOP_INFO(a, A2) strong_dual.setLinearCoef(y[a], info.cost_A2.at(a));
	}
	else {
		LOOP(p, P) strong_dual.setLinearCoef(z[p], null_costs[p]);
	}

	// Dual
	if (dual_space == ARC) {
		strong_dual.setLinearCoef(lambda[prob->commodities[k].origin], -1);
		strong_dual.setLinearCoef(lambda[prob->commodities[k].destination], 1);
	}
	else {
		strong_dual.setLinearCoef(lk, -1);
	}

	cplex_model.add(strong_dual);

	// LINEARIZATION
	if (opt_cond == STRONG_DUAL) {
		// Bilinear 1
		bilinear1 = RangeArray(env, A1, -IloInfinity, 0);
		LOOP_INFO(a, A1) {
			bilinear1[a].setLinearCoef(tx[a], 1);
			if (primal_space == ARC)
				bilinear1[a].setLinearCoef(x[a], -prob->big_m[k][a]);
			else
				LOOP(p, P) if (toll_sets[p].count(a)) bilinear1[a].setLinearCoef(z[p], -prob->big_m[k][a]);
		}

		// Bilinear 2
		bilinear2 = RangeArray(env, A1);
		LOOP_INFO(a, A1) {
			bilinear2[a] = IloRange(env, -IloInfinity, prob->big_n[a]);
			bilinear2[a].setLinearCoef(t[a], 1);
			bilinear2[a].setLinearCoef(tx[a], -1);
			if (primal_space == ARC)
				bilinear2[a].setLinearCoef(x[a], prob->big_n[a]);
			else
				LOOP(p, P) if (toll_sets[p].count(a)) bilinear2[a].setLinearCoef(z[p], prob->big_n[a]);
		}

		// Bilinear 3
		bilinear3 = RangeArray(env, A1, -IloInfinity, 0);
		LOOP_INFO(a, A1) {
			bilinear3[a].setLinearCoef(t[a], -1);
			bilinear3[a].setLinearCoef(tx[a], 1);
		}

		LOOP_INFO(a, A1) {
			cplex_model.add(bilinear1[a]);
			cplex_model.add(bilinear2[a]);
			cplex_model.add(bilinear3[a]);
		}
	}
	// COMPLEMENTARY SLACKNESS
	else {
		// Dual Arc
		if (dual_space == ARC) {
			// Big-M calculation
			lgraph->set_toll_arcs_enabled(true);

			// Set toll to max
			LOOP_INFO(a, A1) {
				auto arc = info.bimap_A1.left.at(a);
				lgraph->edge(arc).toll = prob->big_n[a];
			}
			vector<cost_type> lambda_ub = lgraph->price_to_dst(prob->commodities[k].destination);

			// Set toll to 0
			LOOP_INFO(a, A1) {
				auto arc = info.bimap_A1.left.at(a);
				lgraph->edge(arc).toll = 0;
			}
			vector<cost_type> lambda_lb = lgraph->price_to_dst(prob->commodities[k].destination);

			// Formulation
			comp_slack = RangeArray(env, A, 0, IloInfinity);
			LOOP_INFO(a, A) {
				auto arc = info.bimap_A.left.at(a);
				bool is_tolled = info.is_tolled.at(a);
				cost_type cost = info.cost_A.at(a);

				cost_type big_m = cost - lambda_lb[arc.first] + lambda_ub[arc.second];
				if (is_tolled) big_m += prob->big_n[info.a_to_a1(a)];

				comp_slack[a].setLinearCoef(lambda[arc.first], 1);
				comp_slack[a].setLinearCoef(lambda[arc.second], -1);
				if (is_tolled) comp_slack[a].setLinearCoef(t[info.a_to_a1(a)], -1);

				if (primal_space == ARC) {
					if (is_tolled) comp_slack[a].setLinearCoef(x[info.a_to_a1(a)], -big_m);
					else comp_slack[a].setLinearCoef(y[info.a_to_a2(a)], -big_m);
				}
				else {
					LOOP(p, P) if (arc_sets[p].count(a)) comp_slack[a].setLinearCoef(z[p], -big_m);
				}

				comp_slack[a].setLB(cost - big_m - TOLERANCE);
			}
			LOOP_INFO(a, A) cplex_model.add(comp_slack[a]);
		}
		// Dual Path
		else {
			comp_slack = RangeArray(env, P, 0, IloInfinity);
			LOOP(p, P) {
				// Big-M calculation
				cost_type big_m = null_costs[p] - null_costs[0];
				for (int a : toll_sets[p]) big_m += prob->big_n[a];

				// Formulation
				comp_slack[p].setLinearCoef(lk, 1);
				for (int a : toll_sets[p]) comp_slack[p].setLinearCoef(t[a], -1);
				cost_type rhs = null_costs[p];

				if (primal_space == ARC) {
					for (int a : arc_sets[p]) {
						if (info.is_tolled.at(a)) comp_slack[p].setLinearCoef(x[info.a_to_a1(a)], -big_m);
						else comp_slack[p].setLinearCoef(y[info.a_to_a2(a)], -big_m);
					}
					rhs -= big_m * arc_sets[p].size();
				}
				else {
					comp_slack[p].setLinearCoef(z[p], -big_m);
					rhs -= big_m;
				}

				comp_slack[p].setLB(rhs - TOLERANCE);
			}
			cplex_model.add(comp_slack);
		}
	}
}

std::vector<IloNumVar> general_formulation::get_all_variables()
{
	std::vector<IloNumVar> vars;

	if (primal_space == ARC) {
		LOOP_INFO(a, A1) vars.push_back(x[a]);
		LOOP_INFO(a, A2) vars.push_back(y[a]);
	}
	else LOOP(p, P) vars.push_back(z[p]);

	if (dual_space == ARC) LOOP_INFO(i, V) vars.push_back(lambda[i]);
	else vars.push_back(lk);

	if (opt_cond == STRONG_DUAL) LOOP_INFO(a, A1) vars.push_back(tx[a]);
	else vars.push_back(tk);

	return vars;
}

IloExpr general_formulation::get_obj_expr()
{
	IloExpr expr(env);

	// Strong duality
	if (opt_cond == STRONG_DUAL)
		LOOP_INFO(a, A1) expr.setLinearCoef(tx[a], prob->commodities[k].demand);
	// Complementary slackness
	else
		expr.setLinearCoef(tk, prob->commodities[k].demand);

	return expr;
}

std::vector<std::pair<IloNumVar, IloNum>> general_formulation::path_to_solution(const NumArray& tvals, const std::vector<int>& _)
{
	map<IloNumVar, IloNum> sol;

	lgraph->set_toll_arcs_enabled(true);

	// Find the shortest path
	LOOP_INFO(a, A1) {
		auto arc = info.bimap_A1.left.at(a);
		lgraph->edge(arc).toll = tvals[a] * TOLL_PREFERENCE;		// Prefer tolled arcs
	}
	auto path = lgraph->shortest_path(prob->commodities[k].origin, prob->commodities[k].destination);

	// Get the index of the given path
	auto it = std::find(paths.begin(), paths.end(), path);
	assert(it != paths.end());		// Must be a bilevel feasible path
	int p_best = std::distance(paths.begin(), it);

	// PRIMAL
	if (primal_space == ARC) {
		LOOP_INFO(a, A1) sol.emplace(x[a], 0);
		LOOP_INFO(a, A2) sol.emplace(y[a], 0);

		// Select the arcs in the path
		for (int i = 0; i < path.size() - 1; i++) {
			int a = info.bimap_A.right.at(make_pair(path[i], path[i + 1]));
			bool is_tolled = info.is_tolled.at(a);
			if (is_tolled) sol.at(x[info.a_to_a1(a)]) = 1;
			else sol.at(y[info.a_to_a2(a)]) = 1;
		}
	}
	else {
		LOOP(p, P) sol.emplace(z[p], 0);
		sol.at(z[p_best]) = 1;
	}

	// Set toll to original values
	LOOP_INFO(a, A1) {
		auto arc = info.bimap_A1.left.at(a);
		lgraph->edge(arc).toll = tvals[a];
	}

	// DUAL
	if (dual_space == ARC) {
		// Set lambda to the price to dst
		auto prices = lgraph->price_to_dst(prob->commodities[k].destination);
		LOOP_INFO(i, V) sol.emplace(lambda[i], prices[i]);
	}
	else {
		sol.emplace(lk, lgraph->get_path_cost(path));
	}

	// OPT CONDITION
	if (opt_cond == STRONG_DUAL) {
		LOOP_INFO(a, A1) sol.emplace(tx[a], 0);
		for (int a : toll_sets[p_best])
			sol.at(tx[a]) = tvals[a];
	}
	else {
		cost_type revenue = 0;
		for (int a : toll_sets[p_best])
			revenue += tvals[a];
		sol.emplace(tk, revenue);
	}

	return vector<pair<IloNumVar, IloNum>>(sol.begin(), sol.end());
}