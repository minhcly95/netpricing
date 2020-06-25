#include "light_graph.h"

#include "../macros.h"

#include <algorithm>
#include <queue>
#include <tuple>
#include <set>
#include <iostream>

using namespace std;

using path = light_graph::path;
using iipair = light_graph::iipair;
using toll_set = light_graph::toll_set;
using toll_list = light_graph::toll_list;

vector<path> light_graph::k_shortest_paths(int from, int to, int K, bool toll_free_break)
{
	using cppair = std::pair<cost_type, path>;

	clear_temp_states();

	// List of shortest paths A
	vector<path> A;

	// Heap of candidates B
	std::priority_queue<cppair, vector<cppair>, std::greater<cppair>> B;

	// First shortest path
	path p = shortest_path(from, to);
	if (!p.empty())
		A.emplace_back(std::move(p));
	else
		return A;

	for (int k = 1; k < K; k++) {
		const path& last_path = A.back();

		// Look up for root path mismatch
		vector<bool> root_path_matchable(A.size(), true);

		// For each spur node except the last one
		for (int i = 0; i < last_path.size() - 1; i++) {
			int spur_node = last_path[i];

			// Root path (later total path)
			path new_path(last_path.begin(), last_path.begin() + i + 1);

			// If another shortest path shares the same root path, remove the next edge
			for (int j = 0; j < A.size(); j++) {
				if (!root_path_matchable[j])
					continue;

				const path& p = A[j];

				// if root_path_matchable[j] is true, all previous nodes of A[j] and new_path are matched
				if (p.size() > i&& p[i] == spur_node) {
					edge(p[i], p[i + 1]).temp_enabled = false;
				}
				else {
					root_path_matchable[j] = false;
				}
			}

			// Remove all nodes in the root path except for the spur node
			for (int node : new_path)
				if (node != spur_node)
					temp_enabled_V[node] = false;

			// Calculate the spur path
			path spur_path = shortest_path(spur_node, to);
			if (!spur_path.empty()) {
				new_path.insert(new_path.end(), spur_path.begin() + 1, spur_path.end());
				double cost = get_path_cost(new_path);

				// Duplicated path will be removed when popped
				B.push(make_pair(cost, new_path));
			}

			// Restore the temp states
			clear_temp_states();
		}

		// Break if there are no candidates
		if (B.empty())
			break;

		// Move the best candidate to A (remove all duplicates in the process)
		path best_path = B.top().second;
		while (B.top().second == best_path)
			B.pop();

		// Test for toll-free path
		bool is_toll_free = toll_free_break;
		if (toll_free_break) {
			for (int i = 0; i < best_path.size() - 1; i++) {
				if (edge(best_path[i], best_path[i + 1]).is_tolled) {
					is_toll_free = false;
					break;
				}
			}
		}

		A.emplace_back(std::move(best_path));

		if (is_toll_free)
			break;
	}

	return A;
}

void light_graph::clear_temp_states()
{
	fill(temp_enabled_V.begin(), temp_enabled_V.end(), true);
	for (auto& edge : Eall) edge.temp_enabled = true;
}

vector<path> light_graph::filter_bilevel_feasible(const vector<path>& input) const {
	vector<path> output;
	vector<toll_set> visited_sets;

	for (const path& path : input) {
		// Get the set of toll arcs
		toll_set toll_set = get_toll_set(path);

		// If the toll set is empty, break (because this's the last choice of the commodity)
		if (toll_set.empty()) {
			output.push_back(path);
			break;
		}

		// Check if it is superset of any previous set, if so, remove it
		bool eliminated = std::any_of(visited_sets.begin(), visited_sets.end(),
									  [&](const auto& s) {
										  return std::includes(toll_set.begin(), toll_set.end(), s.begin(), s.end());
									  });

		// Add to visited sets if not eliminated
		if (!eliminated) {
			visited_sets.emplace_back(std::move(toll_set));
			output.push_back(path);
		}
	}

	return output;
}

vector<path> light_graph::bilevel_feasible_paths_yen(int from, int to, int k)
{
	vector<path> paths = toll_unique_paths(from, to, k);
	return filter_bilevel_feasible(paths);
}

vector<path> light_graph::bilevel_feasible_paths(int from, int to, int K)
{
	// An queue entry: cost (for the heap), path, list of toll arcs
	using qentry = std::tuple<cost_type, path, vector<iipair>>;

	clear_temp_states();

	// List of shortest paths A
	vector<qentry> A;
	vector<path> rpaths;

	// Heap of candidates B
	std::priority_queue<qentry, vector<qentry>, std::greater<qentry>> B;

	// First shortest path
	path first_path = shortest_path(from, to);
	if (!first_path.empty()) {
		cost_type cost = get_path_cost(first_path);
		toll_list tlist = get_toll_list(first_path);
		rpaths.push_back(first_path);
		A.emplace_back(cost, std::move(first_path), std::move(tlist));
	}
	else
		return vector<path>();

	while (true) {
		const qentry& last_entry = A.back();
		cost_type last_cost = std::get<0>(last_entry);
		const path& last_path = std::get<1>(last_entry);
		const auto& last_tlist = std::get<2>(last_entry);

		//cout << "entering " << last_tlist << "    " << last_cost << endl;

		// The last path is toll-free path
		if (last_tlist.empty())
			break;

		// Look up for root path mismatch
		vector<bool> root_path_matchable(A.size(), true);

		// For each toll arc in the toll set
		for (int i = -1; i < (int)(last_tlist.size()) - 1; i++) {
			// Get the spur node (the end of the previous toll arc or the origin)
			int spur_node = (i < 0) ? from : last_tlist[i].second;

			// Root path (later total path)
			auto spur_node_it = std::find(last_path.begin(), last_path.end(), spur_node);
			path new_path(last_path.begin(), spur_node_it);

			// If i = -1, remove first toll arc in all paths in A
			if (i < 0) {
				for (int j = 0; j < A.size(); j++) {
					const toll_list& tlist = std::get<2>(A[j]);
					edge(tlist[0]).temp_enabled = false;
				}
			}
			// If another shortest path shares the same sublist of toll arcs up to tlist[i], remove the next toll arc
			else {
				for (int j = 0; j < A.size(); j++) {
					if (!root_path_matchable[j])
						continue;

					const toll_list& tlist = std::get<2>(A[j]);

					// if root_path_matchable[j] is true, all previous toll arcs of A[j] and new_path are matched
					if (tlist.size() > i + 1 && tlist[i] == last_tlist[i]) {
						edge(tlist[i + 1]).temp_enabled = false;
					}
					else {
						root_path_matchable[j] = false;
					}
				}
			}

			// Remove all nodes in the root path except for the spur node
			for (int node : new_path)
				if (node != spur_node)
					temp_enabled_V[node] = false;

			// Calculate the spur path
			path spur_path = shortest_path(spur_node, to);
			if (!spur_path.empty()) {
				new_path.insert(new_path.end(), spur_path.begin(), spur_path.end());
				cost_type cost = get_path_cost(new_path);
				toll_list new_tlist = get_toll_list(new_path);
				//cout << "  adding " << new_tlist << "    " << cost << endl;

				// Duplicated path will be removed when popped
				B.emplace(cost, std::move(new_path), std::move(new_tlist));
			}

			// Restore the temp states
			clear_temp_states();
		}

		// Break if there are no candidates
		if (B.empty())
			break;

		// Move the best candidate to A (remove all duplicates in the process)
		qentry best_entry = B.top();
		while (!B.empty() && B.top() == best_entry)
			B.pop();

		// Check for dominance condition
		const vector<iipair>& best_tlist = std::get<2>(best_entry);
		toll_set best_tset(best_tlist.begin(), best_tlist.end());
		if (all_of(A.begin(), A.end(),
				   [&](const auto& entry) {
					   const vector<iipair>& tlist = std::get<2>(entry);
					   toll_set tset(tlist.begin(), tlist.end());
					   return !std::includes(best_tset.begin(), best_tset.end(), tset.begin(), tset.end());
				   })) {
			rpaths.push_back(std::get<1>(best_entry));
			if (rpaths.size() >= K)
				break;
		}
		/*else {
			cout << "  excluding " << best_tlist << "    " << std::get<0>(best_entry) << endl;
			auto it = find_if(A.begin(), A.end(),
					[&](const auto& entry) {
						const vector<iipair>& tlist = std::get<2>(entry);
						toll_set tset(tlist.begin(), tlist.end());
						return std::includes(best_tset.begin(), best_tset.end(), tset.begin(), tset.end());
					});
			cout << "    superset of " << std::get<2>(*it) << "    " << std::get<0>(*it) << endl;
		}*/

		A.emplace_back(std::move(best_entry));
	}

	return rpaths;
}

vector<path> light_graph::toll_unique_paths(int from, int to, int k)
{
	vector<path> kpaths = k_shortest_paths(from, to, k, true);
	vector<path> rpaths;

	set<toll_set> visited_sets;

	for (const path& path : kpaths) {
		// Get the set of toll arcs
		toll_set toll_set = get_toll_set(path);

		// If the toll set is empty, break (because this's the last choice of the commodity)
		if (toll_set.empty()) {
			rpaths.push_back(path);
			break;
		}

		// Try adding to visited sets
		if (visited_sets.emplace(std::move(toll_set)).second) {
			// If not visited, add to rpaths
			rpaths.push_back(path);
		}
	}

	return rpaths;
}

struct bfpath2_entry {
	vector<int> path;
	cost_type cost;
	int spur_node;
	vector<iipair> removed;
};
struct bfpath2_compare {
	bool operator()(const bfpath2_entry* a, const bfpath2_entry* b) {
	// Reverse the order, smaller is better
		return a->cost > b->cost;
	}
};

vector<path> light_graph::bilevel_feasible_paths_2(int from, int to, int K, bool filter)
{
	clear_temp_states();

	// List of shortest paths A
	vector<path> A;

	// Heap of candidates B
	std::priority_queue<bfpath2_entry*, vector<bfpath2_entry*>, bfpath2_compare> B;

	vector<toll_set> visited_sets;

	// First shortest path
	path first_path = shortest_path(from, to);
	if (!first_path.empty()) {
		cost_type cost = get_path_cost(first_path);
		toll_list tlist = get_toll_list(first_path);
		B.push(new bfpath2_entry {
			.path = first_path,
			.cost = get_path_cost(first_path),
			.spur_node = from,
			.removed = vector<iipair>() });
	}
	// Disconnected
	else
		return vector<path>();

	while (!B.empty() && A.size() < K) {
		// Get the best candidate path
		bfpath2_entry* last_entry = B.top();
		B.pop();

		const path& last_path = last_entry->path;
		cost_type last_cost = last_entry->cost;
		int last_spur_node = last_entry->spur_node;
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
		} else
			A.push_back(last_path);

		auto last_tlist = get_toll_list(last_path);

		// The last path is toll-free path, stop
		if (last_tlist.empty())
			break;

		// Find the first toll arcs succeeding spur_node
		toll_list::iterator it;
		if (last_spur_node == from)
			it = last_tlist.begin();
		else {
			it = std::find_if(last_tlist.begin(), last_tlist.end(),
							  [&](const auto& pair) {
								  return pair.second == last_spur_node;
							  });
			assert(it != last_tlist.end()); // This iterator must exist
			it++;
		}

		int spur_node = last_spur_node;
		cost_type upper_bound = numeric_limits<cost_type>::infinity();

		// Generate subproblems
		for (; it != last_tlist.end(); it++) {
			vector<iipair> curr_removed = last_removed;
			curr_removed.push_back(*it);

			// Copy the subpath
			path curr_path;
			if (spur_node != from) {
				auto spur_it = std::find(last_path.begin(), last_path.end(), spur_node);
				curr_path = vector<int>(last_path.begin(), spur_it);
			}

			// Disable arcs and nodes
			for (auto& pair : curr_removed) {
				edge(pair).temp_enabled = false;
			}

			for (int node : curr_path)
				if (node != spur_node)
					temp_enabled_V[node] = false;

			// Find the second segment
			path spur_path = shortest_path(spur_node, to);
			if (!spur_path.empty()) {
				curr_path.insert(curr_path.end(), spur_path.begin(), spur_path.end());
				cost_type path_cost = get_path_cost(curr_path);

				// Check upper bound, only add if smaller
				if (path_cost < upper_bound) {
					B.push(new bfpath2_entry{
						.path = curr_path,
						.cost = get_path_cost(curr_path),
						.spur_node = spur_node,
						.removed = std::move(curr_removed) });

					// If the spur path is toll-free, set it as the new upper bound
					if (is_toll_free(spur_path)) {
						upper_bound = path_cost;
					}
				}
			}

			// Restore the temp states
			clear_temp_states();

			// Set the next spur node
			spur_node = it->second;
		}

		delete last_entry;
	}

	// Cleaning up
	while (!B.empty()) {
		bfpath2_entry* entry = B.top();
		B.pop();
		delete entry;
	}

	return A;
}

