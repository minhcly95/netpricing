#include "csenum_context.h"

#include "../branchbound/queue_hybrid_impl.h"
#include "../branchbound/bb_context_impl.h"
#include "../macros.h"

using namespace std;

static const double TOLERANCE = 1e-4;

template struct bb_context<csenum_queue>;

csenum_context::csenum_context(IloEnv& _env, const problem& _prob) :
	env(_env), solver(_env, _prob), prob(solver.prob),
	K(solver.K), V(solver.V), A(solver.A), A1(solver.A1), A2(solver.A2)
{
}

bool csenum_context::update_root_bound(node_type* node)
{
	solver.clear_primal_state();
	solver.clear_dual_state();

	solver.solve_primals();
	solver.solve_dual();

	node->primal_objs = solver.get_primal_costs();
	node->dual_obj = solver.get_dual_cost();
	node->bound = node->dual_obj;
	LOOP(k, K) node->bound -= prob.commodities[k].demand * node->primal_objs[k];

	node->paths = solver.get_paths();
	update_slack_map(node);

	update_candidate_list(node);

	return true;
}

bool csenum_context::update_bound(node_type* node, node_type* parent_node, const candidate_type& coor, bool branch_dir)
{
	bool is_feasible;

	if (branch_dir == PRIMAL) {
		solver.push_primal_state(coor);

		int k = coor.k;
		is_feasible = solver.solve_primal(k);
		if (!is_feasible) {
			solver.pop_primal_state();
			return false;
		}

		node->primal_objs = parent_node->primal_objs;
		node->paths = parent_node->paths;

		node->paths[k] = solver.get_path(k);
		node->primal_objs[k] = solver.get_primal_cost(node->paths[k]);

		node->dual_obj = parent_node->dual_obj;
		node->slack_map = parent_node->slack_map;

		solver.pop_primal_state();
	}
	else {
		solver.push_dual_state(coor);
		is_feasible = solver.solve_dual();
		if (!is_feasible) {
			solver.pop_dual_state();
			return false;
		}

		node->primal_objs = parent_node->primal_objs;
		node->paths = parent_node->paths;

		node->dual_obj = solver.get_dual_cost();
		update_slack_map(node);

		solver.pop_dual_state();
	}

	// Update overall bound
	node->bound = node->dual_obj;
	LOOP(k, K) node->bound -= prob.commodities[k].demand * node->primal_objs[k];

	// Update candidate list
	update_candidate_list(node);

	return true;
}

double csenum_context::evaluate_branch(node_type* node, const candidate_type& coor, bool branch_dir)
{
	bool is_feasible;
	double result;

	if (branch_dir == PRIMAL) {
		solver.push_primal_state(coor);

		int k = coor.k;
		is_feasible = solver.solve_primal(coor.k);
		if (!is_feasible) {
			solver.pop_primal_state();
			return -1;
		}

		double new_obj = solver.get_primal_cost(solver.get_path(k));
		result = (new_obj - node->primal_objs[k]) * prob.commodities[k].demand;

		solver.pop_primal_state();
	}
	else {
		solver.push_dual_state(coor);
		is_feasible = solver.solve_dual();
		if (!is_feasible) {
			solver.pop_dual_state();
			return -1;
		}

		double new_obj = solver.get_dual_cost();
		result = node->dual_obj - new_obj;

		solver.pop_dual_state();
	}

	return result;
}

void csenum_context::enter_node(node_type* node)
{
	solver.clear_primal_state();
	solver.clear_dual_state();

	if (node->lineage_node == nullptr)
		return;

	auto lineage = node->lineage_node->get_full_lineage();

	for (const auto& entry : lineage) {
		if (get<1>(entry) == PRIMAL)
			solver.push_primal_state(get<0>(entry));
		else
			solver.push_dual_state(get<0>(entry));
	}
}

void csenum_context::update_slack_map(node_type* node)
{
	node->slack_map = vector<vector<bool>>(K, vector<bool>(A));

	auto lambda = solver.get_lambda();
	auto t = solver.get_t();

	LOOP(k, K) LOOP(a, A) {
		SRC_DST_FROM_A(prob, a);
		cost_type cost = prob.cost_map[edge];
		bool is_tolled = prob.is_tolled_map[edge];

		// Slack calculation
		cost_type slack = cost - lambda[k][src] + lambda[k][dst];
		if (is_tolled)
			slack += t[EDGE_TO_A1(prob, edge)];

		// Set slack map
		node->slack_map[k][a] = (abs(slack) > TOLERANCE);
	}
}

void csenum_context::update_candidate_list(node_type* node)
{
	node->candidates.clear();
	LOOP(k, K) {
		const std::vector<int>& path = node->paths[k];
		for (int i = 0; i < path.size() - 1; ++i) {
			int src = path[i];
			int dst = path[i + 1];
			auto edge = EDGE_FROM_SRC_DST(prob, src, dst);
			int a = EDGE_TO_A(prob, edge);

			if (node->slack_map[k][a]) {
				node->candidates.push_back(csenum_coor{ .k = k, .a = a });
			}
		}
	}
}
