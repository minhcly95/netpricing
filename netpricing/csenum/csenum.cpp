#include "csenum.h"
#include "../macros.h"

#include <chrono>

using namespace std;

csenum::csenum(IloEnv& _env, const problem& _prob) :
	model_single(_prob), primal_lgraph(prob.graph),
	env(_env), dual_model(env), dual_cplex(dual_model),
	queue(), node_count(0), step_count(0)
{
	build_dual_model();

	best_node = csenum_node {
		.cse = this,
		.index = -1,
		.parent = -1,
		.bound = -numeric_limits<double>::infinity()
	};
}

void csenum::build_dual_model()
{
	lambda = VarMatrix(env, K);
	LOOP(k, K) lambda[k] = VarArray(env, V, -IloInfinity, IloInfinity);

	t = VarArray(env, A1, 0, IloInfinity);

	dual_obj = IloMaximize(env);
	LOOP(k, K) {
		commodity& comm = prob.commodities[k];
		dual_obj.setLinearCoef(lambda[k][comm.origin], comm.demand);
		dual_obj.setLinearCoef(lambda[k][comm.destination], -comm.demand);
	}

	dual_feas = RangeMatrix(env, K);
	LOOP(k, K) dual_feas[k] = RangeArray(env, A);
	LOOP(a, A) {
		SRC_DST_FROM_A(prob, a);
		bool is_tolled = prob.is_tolled_map[edge];
		cost_type cost = prob.cost_map[edge];

		LOOP(k, K) {
			dual_feas[k][a] = IloRange(lambda[k][src] - lambda[k][dst] <= cost);
			if (is_tolled)
				dual_feas[k][a].setLinearCoef(t[A_TO_A1(prob, a)], -1);
		}
	}

	dual_model.add(dual_obj);
	LOOP(k, K) dual_model.add(dual_feas[k]);

	dual_cplex.setOut(env.getNullStream());
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
	LOOP(k, K) {
		commodity& comm = prob.commodities[k];
		vector<int> p = primal_lgraph.shortest_path(comm.origin, comm.destination);
		root_node.primal_objs[k] = primal_lgraph.get_path_cost(p) * comm.demand;
		root_node.paths.emplace_back(std::move(p));
	}

	// Dual
	dual_cplex.solve();
	root_node.dual_obj = dual_cplex.getObjValue();
	root_node.update_bound();

	root_node.lambda = NumMatrix(env, K);
	LOOP(k, K) {
		root_node.lambda[k] = NumArray(env, V);
		dual_cplex.getValues(lambda[k], root_node.lambda[k]);
	}

	root_node.t = NumArray(env, A1);
	dual_cplex.getValues(t, root_node.t);

	// Update violated constraints and feasibility test
	root_node.update_violated();

	if (root_node.is_feasible())
		best_node = root_node;
	else
		queue.emplace(std::move(root_node));

	// Statistics
	node_count++;
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

	printf("Node %2d : %2d - bound %5.0f - depth %2d - violated %2d", node.index, node.parent, node.bound, node.get_depth(), node.get_num_violated());

	cout << endl;
	print_lineage(node.lineage);
	cout << endl;

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

	set_dual_state(dual_coors);
	LOOP(k, K) best_lambda[k] = NumArray(env, V);

	LOOP(k, K) {
		commodity& comm = prob.commodities[k];
		const vector<int>& vk = node.violated[k];
		if (vk.empty())
			continue;

		// Find the best improvement
		set_primal_state(primal_coors[k]);
		for (int a : vk) {
			printf("  %2d.%2d ", k, a);
			// Primal improvement
			push_primal_state(a);
			auto path = primal_lgraph.shortest_path(comm.origin, comm.destination);
			bool primal_feasible = !path.empty();
			double primal_impr = -1;
			if (primal_feasible) {
				double cost = primal_lgraph.get_path_cost(path) * comm.demand;
				primal_impr = cost - node.primal_objs[k];
			}
			pop_primal_state();
			printf("- P = %f ", primal_impr);

			// Early break
			if (primal_impr >= 0 && primal_impr <= best_impr) {
				cout << endl;
				continue;
			}

			// Dual improvement
			push_dual_state(csenum_coor{ .k = k, .a = a });
			bool dual_feasible = dual_cplex.solve();
			double dual_impr = dual_feasible ? node.dual_obj - dual_cplex.getObjValue() : -1;
			printf("- D = %f ", dual_impr);
			cout << endl;

			// Overall improvement (min of the two impr-s)
			double impr = (primal_impr < dual_impr && primal_impr >= 0) ? primal_impr : dual_impr;
			if (impr > best_impr && impr >= 0) {
				best_impr = impr;
				best_primal_impr = primal_impr;
				best_dual_impr = dual_impr;
				best_coor.k = k;
				best_coor.a = a;
				best_path = path;
				if (dual_impr >= 0) {
					LOOP(k, K) dual_cplex.getValues(lambda[k], best_lambda[k]);
					dual_cplex.getValues(t, best_t);
				}
			}
			pop_dual_state();
		}
	}

	printf("  best impr %f ", best_impr);
	cout << endl;

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
			if (primal_node.bound > best_node.bound)
				best_node = primal_node;
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
			if (dual_node.bound > best_node.bound)
				best_node = dual_node;
		}
		else {
			queue.emplace(std::move(dual_node));
		}
	}

	// Statistics
	step_count++;
}

void csenum::clear_primal_state()
{
	// Enable all edges in primal graph
	LOOP(i, V) {
		for (auto& pair : primal_lgraph.E[i]) {
			pair.second.enabled = true;
		}
	}
	primal_state_stack.clear();
}

void csenum::clear_dual_state()
{
	// Clear all lower bounds in dual model
	LOOP(k, K) LOOP(a, A) {
		dual_feas[k][a].setLB(-IloInfinity);
	}
	dual_state_stack.clear();
}

void csenum::set_primal_state(std::vector<int> as)
{
	clear_primal_state();

	// Disable all edges listed in coork
	for (int a : as)
		push_primal_state(a);
}

void csenum::set_dual_state(std::vector<csenum_coor> coors)
{
	clear_dual_state();

	// Restrict dual_feas[k][a] to equality
	for (auto& coor : coors)
		push_dual_state(coor);
}

void csenum::push_primal_state(int a)
{
	SRC_DST_FROM_A(prob, a);
	primal_lgraph.E[src][dst].enabled = false;
	primal_state_stack.push_back(a);
}

void csenum::push_dual_state(const csenum_coor& coor)
{
	dual_feas[coor.k][coor.a].setLB(dual_feas[coor.k][coor.a].getUB());
	dual_state_stack.push_back(coor);
}

void csenum::pop_primal_state()
{
	int a = primal_state_stack.back();

	SRC_DST_FROM_A(prob, a);
	primal_lgraph.E[src][dst].enabled = true;

	primal_state_stack.pop_back();
}

void csenum::pop_dual_state()
{
	csenum_coor& coor = dual_state_stack.back();

	dual_feas[coor.k][coor.a].setLB(-IloInfinity);

	dual_state_stack.pop_back();
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

bool csenum::solve()
{
	auto start = chrono::high_resolution_clock::now();

	// Solve root node
	solve_root();

	// Step until queue empty
	while (!queue.empty()) {
		if (get_best_bound() <= get_best_obj())
			break;
		step();
	}

	auto end = chrono::high_resolution_clock::now();
	time = chrono::duration<double>(end - start).count();

	return true;
}

solution csenum::get_solution()
{
	solution sol;
	sol.paths = best_node.paths;
	LOOP(a, A1) {
		sol.tolls.push_back(best_node.t[a]);
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

IloCplex csenum::get_cplex()
{
	return dual_cplex;
}
