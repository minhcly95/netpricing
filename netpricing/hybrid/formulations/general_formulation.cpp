#include "general_formulation.h"

#include "../base/hybrid_model.h"
#include "../../macros.h"
#include "../../utilities/set_var_name.h"
#include "../base/processed_var_name.h"
#include "../../utilities/cplex_compare.h"
#include "../../graph/light_graph.h"
#include "../../models/model_utils.h"

#include <tuple>

using namespace std;

general_formulation::general_formulation(const std::vector<path>& paths,
										 const light_graph& original,
										 space primal_space, space dual_space,
										 opt_condition opt_cond,
										 linearization_method linearization) :
	paths(paths), P(paths.size()), original(original), preproc(paths),
	primal_space(primal_space), dual_space(dual_space), opt_cond(opt_cond), linearization(linearization)
{
	if (opt_cond == STRONG_DUAL && linearization == SUBSTITUTION)
		throw invalid_argument("cannot have both strong duality opt condition and substitution linearization");
}

general_formulation::general_formulation(const std::vector<path>& paths, const light_graph& original, const std::string& code) :
	paths(paths), P(paths.size()), original(original), preproc(paths)
{
	auto model = std::tie(primal_space, dual_space, opt_cond, linearization);

	if (code == "std")
		model = make_tuple(ARC, ARC, STRONG_DUAL, DIRECT);
	else if (code == "apstd")
		model = make_tuple(PATH, ARC, STRONG_DUAL, DIRECT);
	else if (code == "vf")
		model = make_tuple(ARC, PATH, STRONG_DUAL, DIRECT);
	else if (code == "pvf")
		model = make_tuple(PATH, PATH, STRONG_DUAL, DIRECT);
	else if (code == "cs1")
		model = make_tuple(ARC, ARC, COMP_SLACK, DIRECT);
	else if (code == "apcs1")
		model = make_tuple(PATH, ARC, COMP_SLACK, DIRECT);
	else if (code == "vfcs1")
		model = make_tuple(ARC, PATH, COMP_SLACK, DIRECT);
	else if (code == "pcs1")
		model = make_tuple(PATH, PATH, COMP_SLACK, DIRECT);
	else if (code == "cs2")
		model = make_tuple(ARC, ARC, COMP_SLACK, SUBSTITUTION);
	else if (code == "apcs2")
		model = make_tuple(PATH, ARC, COMP_SLACK, SUBSTITUTION);
	else if (code == "vfcs2")
		model = make_tuple(ARC, PATH, COMP_SLACK, SUBSTITUTION);
	else if (code == "pcs2")
		model = make_tuple(PATH, PATH, COMP_SLACK, SUBSTITUTION);
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
	info = preproc.preprocess_impl(original, prob->commodities[k], k);

	V = *(info.V.rbegin()) + 1;
	A = *(info.A.rbegin()) + 1;
	A1 = *(info.A1.rbegin()) + 1;
	A2 = *(info.A2.rbegin()) + 1;
	lgraph = new light_graph(info.build_graph());

	// Process path
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

	// Add a little penalty to favor path with fewest arcs
	/*LOOP_INFO(a, A2) {
		auto arc = info.bimap_A2.left.at(a);
		lgraph->edge(arc).toll = TOLLFREE_PENALTY;
	}*/
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
			y = VarArray(env, A2, 0, 1);

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
		lk = IloNumVar(env, 0, IloInfinity);
		SET_VAR_NAMES_K(model, k, lk);
		cplex_model.add(lk);
	}

	// Direct linearization
	if (linearization == DIRECT) {
		tx = VarArray(env, A1, 0, IloInfinity);
		SET_VAR_NAMES_K(info, k, tx);
		LOOP_INFO(a, A1) cplex_model.add(tx[a]);
	}
	// Substitution linearization
	else {
		tk = IloNumVar(env, 0, IloInfinity);
		SET_VAR_NAMES_K(model, k, tk);
		cplex_model.add(tk);
	}

	// OBJECTIVE
	// Direct linearization
	if (linearization == DIRECT)
		LOOP_INFO(a, A1) obj.setLinearCoef(tx[a], prob->commodities[k].demand);
	// Substitution linearization
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
	bool use_strong_dual = false;
	strong_dual = IloRange(env, 0, TOLERANCE);

	if (opt_cond == STRONG_DUAL) {
		LOOP_INFO(a, A1) strong_dual.setLinearCoef(tx[a], 1);
		use_strong_dual = true;
	}
	else if (linearization == SUBSTITUTION) {
		strong_dual.setLinearCoef(tk, 1);
		use_strong_dual = true;
	}

	if (use_strong_dual) {
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
	}

	// DIRECT LINEARIZATION
	if (linearization == DIRECT) {
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
	if (opt_cond == COMP_SLACK) {
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
					rhs -= (big_m + TOLERANCE) * arc_sets[p].size();
				}
				else {
					comp_slack[p].setLinearCoef(z[p], -big_m);
					rhs -= (big_m + TOLERANCE);
				}

				comp_slack[p].setLB(rhs);
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

	if (linearization == DIRECT) LOOP_INFO(a, A1) vars.push_back(tx[a]);
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

std::vector<int> general_formulation::get_optimal_path()
{
	// Primal result
	path primal_path;
	if (primal_space == ARC) {
		// Src-dst map
		multimap<int, int> src_dst_map;
		LOOP_INFO(a, A1) {
			auto arc = info.bimap_A1.left.at(a);
			if (model->cplex.getValue(x[a]) > 0.5)
				src_dst_map.emplace(arc.first, arc.second);
		}
		LOOP_INFO(a, A2) {
			auto arc = info.bimap_A2.left.at(a);
			if (model->cplex.getValue(y[a]) > 0.5)
				src_dst_map.emplace(arc.first, arc.second);
		}

		primal_path = extract_path_from_src_dst_map(src_dst_map);
	}
	else {
		NumArray zvals = get_values(model->cplex, z);
		LOOP(p, P)
			if (zvals[p] > 0.5)
				primal_path = paths[p];
		assert(!primal_path.empty());
	}

	//cerr << "Comm " << k << " Primal Matched ";
	//for (int i : primal_path) cerr << i << " ";
	//cerr << endl;

	// Dual result (to verify)
	//if (dual_space == ARC) {
	//}
	//else {
	//	LOOP(p, P) {
	//		double expr_val = model->cplex.getValue(valuefunc[p].getExpr());
	//		if (expr_val >= valuefunc[p].getUB() - TOLERANCE) {
	//			cerr << "Comm " << k << " Dual Matched ";
	//			for (int i : paths[p]) cerr << i << " ";
	//			cerr << endl;
	//		}
	//		expr_val = model->cplex.getValue(comp_slack[p].getExpr());
	//		if (expr_val <= comp_slack[p].getLB() + TOLERANCE * 10) {
	//			cerr << "Comm " << k << " CS Matched ";
	//			for (int i : paths[p]) cerr << i << " ";
	//			cerr << endl;
	//		}
	//	}
	//}

	// Reverse preprocessing
	light_graph original(prob->graph);
	original.set_toll_arcs_enabled(false);

	auto toll_arcs = lgraph->get_toll_list(primal_path);
	set<int> toll_heads;
	toll_heads.insert(prob->commodities[k].destination);

	// Extract toll heads
	for (const auto& pair : toll_arcs) {
		auto edge = EDGE_FROM_SRC_DST(*prob, pair.first, pair.second);
		toll_heads.insert(pair.first);
	}

	path new_path;
	auto it = primal_path.begin();

	while (it != primal_path.end()) {
		int src = *it;
		it = find_if(it, primal_path.end(),
					 [&](int i) {
						 return toll_heads.count(i);
					 });
		assert(it != primal_path.end());
		int dst = *it;

		if (src == dst) {
			new_path.push_back(src);
		}
		else {
			auto segment = original.shortest_path(src, dst);
			assert(segment.size());
			new_path.insert(new_path.end(), segment.begin(), segment.end());
		}

		it++;
	}

	return new_path;
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

	// LINEARIZATION
	if (linearization == DIRECT) {
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

bool general_formulation::has_callback()
{
	return check_model(ARC, PATH, COMP_SLACK);
}

void general_formulation::invoke_callback(const IloCplex::Callback::Context& context, const NumArray& tvals)
{
	// Specialized for VFCS models
	assert(check_model(ARC, PATH, COMP_SLACK));

	// Src-dst map
	multimap<int, int> src_dst_map;
	LOOP_INFO(a, A1) {
		auto arc = info.bimap_A1.left.at(a);
		if (context.getCandidateValue(x[a]) > 0.5)
			src_dst_map.emplace(arc.first, arc.second);
	}
	LOOP_INFO(a, A2) {
		auto arc = info.bimap_A2.left.at(a);
		if (context.getCandidateValue(y[a]) > 0.5)
			src_dst_map.emplace(arc.first, arc.second);
	}

	// Get the current path
	auto path = extract_path_from_src_dst_map(src_dst_map);

	// Check if it is bilevel feasible
	bool bifeas = !path.empty() &&
		std::any_of(paths.begin(), paths.end(),
					[&](const vector<int>& p) {
						return path == p;
					});

	// Build a cut if it is not
	if (!bifeas) {
		//cerr << "Comm " << k << " Reject ";
		//for (auto& pair : src_dst_map) {
		//	cerr << "(" << pair.first << "->" << pair.second << ") ";
		//}
		//cerr << endl;

		// Big-M calculation
		cost_type path_null_cost = 0;
		set<pair<int, int>> path_toll_set;
		for (auto& arc : src_dst_map) {
			auto& edge = lgraph->edge(arc);
			path_null_cost += edge.cost;
			if (edge.is_tolled) path_toll_set.insert(arc);
		}

		cost_type big_m = path_null_cost - null_costs[0];
		for (auto& arc : path_toll_set) {
			big_m += prob->big_n[info.bimap_A1.right.at(arc)];
		}

		// Cut Formulation
		IloRange cut(env, 0, IloInfinity);

		cut.setLinearCoef(lk, 1);
		for (auto& arc : path_toll_set) cut.setLinearCoef(t[info.bimap_A1.right.at(arc)], -1);

		for (auto& arc : src_dst_map) {
			int a = info.bimap_A.right.at(arc);
			if (info.is_tolled.at(a)) cut.setLinearCoef(x[info.a_to_a1(a)], -big_m);
			else cut.setLinearCoef(y[info.a_to_a2(a)], -big_m);
		}

		cost_type rhs = path_null_cost;
		rhs -= (big_m + TOLERANCE) * src_dst_map.size();

		cut.setLB(rhs);

		context.rejectCandidate(cut);
		cut.end();
	}
	//else {
	//	cerr << "Comm " << k << " Accept ";
	//	for (int i : path) {
	//		cerr << i << " ";
	//	}
	//	cerr << endl;
	//}
}

std::vector<int> general_formulation::extract_path_from_src_dst_map(const std::multimap<int, int>& src_dst_map)
{
	// Build a path
	vector<int> path;
	set<int> visited;
	multimap<int, int> map(src_dst_map);

	auto& comm = prob->commodities[k];
	int current = comm.origin;
	path.push_back(current);
	visited.insert(current);

	while (current != comm.destination) {
		auto it = map.find(current);
		current = it->second;

		// A loop
		if (visited.count(current)) {
			auto loop_it = std::find(path.begin(), path.end(), current);
			path.erase(loop_it, path.end());
		}

		path.push_back(current);
		visited.insert(current);

		map.erase(it);
	}

	// Test if cyclic, return empty
	if (path.size() - 1 != src_dst_map.size()) {
		return vector<int>();
	}

	return path;
}
