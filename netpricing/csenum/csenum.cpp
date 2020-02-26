#include "csenum.h"
#include "../macros.h"

#include <chrono>

using namespace std;

csenum::csenum(IloEnv& _env, const problem& _prob) :
	env(_env), solver(_env, _prob), prob(solver.prob),
	K(solver.K), V(solver.V), A(solver.A), A1(solver.A1), A2(solver.A2),
	queue(), node_count(0), step_count(0), time_limit(0)
{
	best_node = csenum_node {
		.cse = this,
		.index = -1,
		.parent = -1,
		.bound = -numeric_limits<double>::infinity()
	};
}

void csenum::solve_root()
{
	csenum_node root_node{
		.cse = this,
		.index = 0,
		.parent = -1,
		.bound = 0,
		.primal_objs = vector<double>(K),
	};

	// Primal
	LOOP(k, solver.K) {
		commodity& comm = prob.commodities[k];
		solver.solve_primal(k);
		root_node.primal_objs[k] = solver.get_primal_cost() * comm.demand;
		root_node.paths.push_back(solver.get_path());
	}

	// Dual
	solver.solve_dual();
	root_node.dual_obj = solver.get_dual_cost();
	root_node.update_bound();

	root_node.lambda = solver.get_lambda();
	root_node.t = solver.get_t();

	// Update violated constraints and feasibility test
	root_node.update_violated();
	print_node(root_node);

	if (root_node.is_feasible())
		best_node = root_node;
	else
		queue.emplace(std::move(root_node));

	// Statistics
	node_count++;
	step_count++;
}

void print_lineage(const vector<csenum_branch>& lineage) {
	for (const auto& branch : lineage) {
		printf("%d.%d%s ", branch.k, branch.a, branch.dir ? "D" : "P");
	}
}

void csenum::step()
{
	// Get the top node
	csenum_node node = std::move(queue.top());
	queue.pop();

	if (step_count % 100 == 0) {
		print_node(node);
	}

	//printf("Node %2d : %2d - bound %5.0f - depth %2d - violated %2d", node.index, node.parent, node.bound, node.get_depth(), node.get_num_violated());

	//cout << endl;
	//print_lineage(node.lineage);
	//cout << endl;

	// Process lineage
	vector<vector<int>> primal_coors(K);
	vector<csenum_coor> dual_coors;
	for (const auto& branch : node.lineage) {
		if (branch.dir == PRIMAL) {
			primal_coors[branch.k].push_back(branch.a);
		}
		else {
			dual_coors.push_back(csenum_coor{ .k = branch.k,.a = branch.a });
		}
	}

	// Strong branching
	csenum_coor best_coor{ .k = -1, .a = -1 };
	double best_primal_impr = -1;
	double best_dual_impr = -1;
	double best_impr = -1;
	vector<int> best_path;
	NumMatrix best_lambda(env, K);
	NumArray best_t(env, A1);

	solver.set_dual_state(dual_coors);
	LOOP(k, K) best_lambda[k] = NumArray(env, V);

	LOOP(k, K) {
		commodity& comm = prob.commodities[k];
		const vector<int>& vk = node.violated[k];
		if (vk.empty())
			continue;

		// Find the best improvement
		solver.set_primal_state(primal_coors[k]);
		for (int a : vk) {
			//printf("  %2d.%2d ", k, a);
			// Primal improvement
			solver.push_primal_state(a);
			bool primal_feasible = solver.solve_primal(k);

			vector<int> path;
			double primal_impr = -1;

			if (primal_feasible) {
				double cost = solver.get_primal_cost() * comm.demand;
				primal_impr = cost - node.primal_objs[k];
			}
			solver.pop_primal_state();
			//printf("- P = %f ", primal_impr);

			// Early break
			if (primal_impr >= 0 && primal_impr <= best_impr) {
				//cout << endl;
				continue;
			}

			// Dual improvement
			solver.push_dual_state(csenum_coor{ .k = k, .a = a });
			bool dual_feasible = solver.solve_dual();
			double dual_impr = dual_feasible ? node.dual_obj - solver.get_dual_cost() : -1;
			//printf("- D = %f ", dual_impr);
			//cout << endl;

			// Overall improvement (min of the two impr-s)
			double impr = (primal_impr < dual_impr && primal_impr >= 0) ? primal_impr : dual_impr;
			if (impr > best_impr && impr >= 0) {
				best_impr = impr;
				best_primal_impr = primal_impr;
				best_dual_impr = dual_impr;
				best_coor.k = k;
				best_coor.a = a;
				best_path = solver.get_path();
				if (dual_impr >= 0) {
					solver.get_lambda(best_lambda);
					solver.get_t(best_t);
				}
			}
			solver.pop_dual_state();
		}
	}

	//printf("  best impr %f ", best_impr);
	//cout << endl;

	// Primal node (only if primal is feasible)
	if (best_primal_impr >= 0) {
		csenum_node primal_node(node);
		primal_node.index = node_count++;
		primal_node.parent = node.index;
		primal_node.primal_objs[best_coor.k] += best_primal_impr;
		primal_node.update_bound();
		primal_node.paths[best_coor.k] = best_path;
		primal_node.lineage.push_back(csenum_branch{
			.k = best_coor.k,
			.a = best_coor.a,
			.dir = PRIMAL
									  });
		primal_node.update_violated(best_coor.k);

		if (primal_node.is_feasible()) {
			if (primal_node.bound > best_node.bound) {
				best_node = primal_node;
				print_node(primal_node, true);
			}
		}
		else {
			queue.emplace(std::move(primal_node));
		}
	}

	// Dual node (only if dual is feasible)
	if (best_dual_impr >= 0) {
		csenum_node dual_node(node);
		dual_node.index = node_count++;
		dual_node.parent = node.index;
		dual_node.dual_obj -= best_dual_impr;
		dual_node.update_bound();
		dual_node.lambda = best_lambda;
		dual_node.t = best_t;
		dual_node.lineage.push_back(csenum_branch{
			.k = best_coor.k,
			.a = best_coor.a,
			.dir = DUAL
									});
		dual_node.update_violated();

		if (dual_node.is_feasible()) {
			if (dual_node.bound > best_node.bound) {
				best_node = dual_node;
				print_node(dual_node, true);
			}
		}
		else {
			queue.emplace(std::move(dual_node));
		}
	}

	// Statistics
	step_count++;
}

double csenum::get_best_obj()
{
	return best_node.bound;
}

double csenum::get_best_bound()
{
	if (!queue.empty())
		return queue.top().bound;
	else
		return -numeric_limits<double>::infinity();
}

void csenum::print_node(const csenum_node& node, bool feasible)
{
	double best_obj = get_best_obj();
	double gap = 100 * (node.bound - best_obj) / best_obj;

	char last_branch_str[25];
	if (!node.lineage.empty()) {
		const auto& last_branch = node.lineage.back();
		SRC_DST_FROM_A(solver.prob, last_branch.a);
		bool is_tolled = solver.prob.is_tolled_map[edge];
		sprintf(last_branch_str, "%s[%d,%d->%d] %s", is_tolled ? "x" : "y", last_branch.k, src, dst, last_branch.dir ? "D" : "P");
	}
	else
		last_branch_str[0] = '\0';

	auto now = std::chrono::high_resolution_clock::now();
	double curr_time = std::chrono::duration<double>(now - start_time).count();

	printf("%s%5d %5d %5ld %5d %6d %5d %4d %10.3f %10.3f %5.2f %15s %7.1f",
		   feasible ? "*" : " ",
		   node_count,
		   step_count,
		   queue.size(),
		   node.index,
		   node.parent,
		   node.get_depth(),
		   node.get_num_violated(),
		   node.bound,
		   best_obj,
		   gap,
		   last_branch_str,
		   curr_time
	);
	cout << endl;
}

bool csenum::solve_impl()
{
	// Print header
	printf("  Node  Step  Left Index Parent Depth Vltd      Bound    BestObj   Gap      LastBranch    Time");
	cout << endl;

	// Solve root node
	solve_root();

	// Step until queue empty
	while (!queue.empty()) {
		if (get_best_bound() <= get_best_obj())
			break;
		step();

		if (time_limit > 0) {
			auto now = std::chrono::high_resolution_clock::now();
			double curr_time = std::chrono::duration<double>(now - start_time).count();
			if (curr_time >= time_limit)
				break;
		}
	}

	return true;
}

void csenum::config(const model_config& config)
{
	time_limit = config.time_limit;
}

solution csenum::get_solution()
{
	solution sol;
	if (best_node.index >= 0) {
		sol.paths = best_node.paths;
		LOOP(a, A1) {
			sol.tolls.push_back(best_node.t[a]);
		}
	}
	return sol;
}

string csenum::get_report()
{
	ostringstream ss;
	ss << "OBJ: " << get_best_obj() << endl <<
		"TIME: " << get_time() << " s" << endl;
	return ss.str();
}
