#include "cplex_graph.h"
#include "../macros.h"

#include <algorithm>
#include <queue>
#include <set>

using namespace std;
using namespace boost;

using path = light_graph::path;
using iipair = light_graph::iipair;
using toll_set = light_graph::toll_set;
using toll_list = light_graph::toll_list;

cplex_graph::cplex_graph(const problem_base::graph_type& graph) :
	light_graph(graph), env(), model(env)
{
	// Variables
	x = NumVarArray(env, num_edges(graph), 0, 1);

	// Objective
	obj = IloMinimize(env);
	LOOP(a, Eall.size()) {
		obj.setLinearCoef(x[a], Eall[a].cost);		
	}

	// Flow constraints
	flow_constr = RangeArray(env, V, 0, 0);

	// Flow matrix
	LOOP(a, Eall.size()) {
		flow_constr[Eall[a].src].setLinearCoef(x[a], 1);
		flow_constr[Eall[a].dst].setLinearCoef(x[a], -1);
	}

	// Add to model
	model.add(obj);
	model.add(flow_constr);

	cplex = IloCplex(model);
	cplex.setOut(env.getNullStream());
}

path cplex_graph::shortest_path_cplex(int from, int to)
{
	// Reset origin/destination
	LOOP(i, V) flow_constr[i].setBounds(0, 0);
	flow_constr[from].setBounds(1, 1);
	flow_constr[to].setBounds(-1, -1);

	// Solve the problem
	if (cplex.solve()) {
		// Extract result
		NumArray xvals(env);
		cplex.getValues(x, xvals);

		// Map src -> dst
		map<int, int> src_dst_map;
		vector<light_edge> elist;
		LOOP(a, Eall.size()) {
			if (xvals[a] > 0.5) {
				src_dst_map[Eall[a].src] = Eall[a].dst;
				elist.push_back(Eall[a]);
			}
		}

		// Trace the path
		int curr = from;
		path p;
		while (curr != to) {
			p.push_back(curr);
			curr = src_dst_map[curr];
		}
		p.push_back(to);

		xvals.end();
		return p;
	}
	else
		return path();		// Infeasible
}

void cplex_graph::reset()
{
	LOOP(a, Eall.size()) x[a].setBounds(0, 1);
}

void cplex_graph::force(int src, int dst)
{
	x[E[src].at(dst)].setBounds(1, 1);
}

void cplex_graph::disable(int src, int dst)
{
	x[E[src].at(dst)].setBounds(0, 0);
}

struct bfpath3_entry {
	vector<int> path;
	cost_type cost;
	set<iipair> forced;
	vector<iipair> removed;
};
struct bfpath3_compare {
	bool operator()(const bfpath3_entry* a, const bfpath3_entry* b) {
	// Reverse the order, smaller is better
		return a->cost > b->cost;
	}
};

vector<path> cplex_graph::bilevel_feasible_paths_3(int from, int to, int K, bool filter)
{
	clear_temp_states();

	// List of shortest paths A
	vector<path> A;

	// Heap of candidates B
	std::priority_queue<bfpath3_entry*, vector<bfpath3_entry*>, bfpath3_compare> B;

	vector<toll_set> visited_sets;

	// First shortest path
	path first_path = shortest_path(from, to);
	if (!first_path.empty()) {
		cost_type cost = get_path_cost(first_path);
		toll_list tlist = get_toll_list(first_path);
		B.push(new bfpath3_entry{
			.path = first_path,
			.cost = get_path_cost(first_path),
			.forced = set<iipair>(),
			.removed = vector<iipair>() });
	}
	// Disconnected
	else
		return vector<path>();

	while (!B.empty() && A.size() < K) {
		// Get the best candidate path
		bfpath3_entry* last_entry = B.top();
		B.pop();

		const path& last_path = last_entry->path;
		cost_type last_cost = last_entry->cost;
		const auto& last_forced = last_entry->forced;
		const auto& last_removed = last_entry->removed;

		if (filter) {
			// Check if it is superset of any previous set
			toll_set tset = get_toll_set(last_path);
			bool dominated = std::any_of(visited_sets.begin(), visited_sets.end(),
										 [&](const auto& s) {
											 return std::includes(tset.begin(), tset.end(), s.begin(), s.end());
										 });
		   // Add to output if not dominated
			if (!dominated) {
				A.push_back(last_path);
				visited_sets.emplace_back(std::move(tset));
			}
		}
		else
			A.push_back(last_path);

		auto last_tlist = get_toll_list(last_path);

		// The last path is toll-free path, stop
		if (last_tlist.empty())
			break;
		
		// Find the set of non-forced tolled arcs
		last_tlist.erase(remove_if(last_tlist.begin(), last_tlist.end(),
								   [&](const auto& pair) {
									   return last_forced.count(pair);
								   }), last_tlist.end());

		// Generate subproblems
		for (int i = 0; i < last_tlist.size(); i++) {
			// Add arcs to forced/removed set
			set<iipair> curr_forced = last_forced;
			for (int j = 0; j < i; j++)
				curr_forced.insert(last_tlist[j]);

			vector<iipair> curr_removed = last_removed;
			curr_removed.push_back(last_tlist[i]);

			// Force/Disable
			for (auto& pair : curr_forced)
				force(pair.first, pair.second);

			for (auto& pair : curr_removed)
				disable(pair.first, pair.second);

			// Solve
			path curr_path = shortest_path_cplex(from, to);
			if (!curr_path.empty()) {
				B.push(new bfpath3_entry{
					.path = curr_path,
					.cost = get_path_cost(curr_path),
					.forced = std::move(curr_forced),
					.removed = std::move(curr_removed) });
			}

			// Reset temp states
			reset();
		}

		delete last_entry;
	}

	// Cleaning up
	while (!B.empty()) {
		bfpath3_entry* entry = B.top();
		B.pop();
		delete entry;
	}

	return A;
}
