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

light_graph::light_graph(const problem_base::graph_type& graph) :
	V(boost::num_vertices(graph)), Eall(), E(V), Er(V),
	temp_enabled_V(V, true)
{
	auto cost_map = boost::get(boost::edge_weight, graph);
	auto is_tolled_map = boost::get(edge_tolled, graph);

	problem_base::edge_iterator ei, ei_end;
	for (tie(ei, ei_end) = boost::edges(graph); ei != ei_end; ++ei) {
		int src = boost::source(*ei, graph);
		int dst = boost::target(*ei, graph);
		cost_type cost = cost_map[*ei];
		bool is_tolled = is_tolled_map[*ei];

		Eall.emplace_back(light_edge{
			.src = src,
			.dst = dst,
			.cost = cost,
			.is_tolled = is_tolled,
			.toll = 0,
			.enabled = true,
			.temp_enabled = true
							   });
	}

	for (auto& edge : Eall) {
		E[edge.src].emplace(edge.dst, edge);
		Er[edge.dst].emplace(edge.src, edge);
	}
}

light_edge& light_graph::edge(int src, int dst)
{
	return E[src].at(dst);
}

light_edge& light_graph::edge(const iipair& pair)
{
	return E[pair.first].at(pair.second);
}

void light_graph::set_toll_arcs_enabled(bool enabled)
{
	for (auto& edge : Eall) {
		if (edge.is_tolled)
			edge.enabled = enabled;
	}
}

void light_graph::clear_toll()
{
	for (auto& edge : Eall) {
		if (edge.is_tolled)
			edge.toll = 0;
	}
}

light_graph::path light_graph::shortest_path(int from, int to)
{
	vector<int> parents;
	vector<cost_type> distances;

	// If "to" is not reached, return empty path
	if (!dijkstra(from, distances, parents, to))
		return path();

	// Trace back the path
	path p;
	int curr = to;
	while (curr != from) {
		p.push_back(curr);
		curr = parents[curr];
	}
	p.push_back(from);
	std::reverse(p.begin(), p.end());

	return p;
}

double light_graph::get_path_cost(const path& p)
{
	double sum = 0;

	for (int i = 0; i < p.size() - 1; i++) {
		const auto& edge = this->edge(p[i], p[i + 1]);
		sum += edge.cost + edge.toll;
	}

	return sum;
}

toll_set light_graph::get_toll_set(const path& p)
{
	toll_set toll_set;
	for (int i = 0; i < p.size() - 1; i++)
		if (edge(p[i], p[i + 1]).is_tolled)
			toll_set.emplace(p[i], p[i + 1]);
	return toll_set;
}

toll_list light_graph::get_toll_list(const path& p)
{
	toll_list toll_list;
	for (int i = 0; i < p.size() - 1; i++)
		if (edge(p[i], p[i + 1]).is_tolled)
			toll_list.emplace_back(p[i], p[i + 1]);
	return toll_list;
}

bool light_graph::is_toll_free(const path& p)
{
	for (int i = 0; i < p.size() - 1; i++)
		if (edge(p[i], p[i + 1]).is_tolled)
			return false;
	return true;
}

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
				if (p.size() > i && p[i] == spur_node) {
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

vector<path> light_graph::bilevel_feasible_paths_yen(int from, int to, int k)
{
	vector<path> kpaths = toll_unique_paths(from, to, k);
	vector<path> rpaths;

	vector<toll_set> visited_sets;

	for (const path& path : kpaths) {
		// Get the set of toll arcs
		toll_set toll_set = get_toll_set(path);

		// If the toll set is empty, break (because this's the last choice of the commodity)
		if (toll_set.empty()) {
			rpaths.push_back(path);
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
			rpaths.push_back(path);
		}
	}

	return rpaths;
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

vector<cost_type> light_graph::price_from_src(int src)
{
	vector<int> parents;
	vector<cost_type> distances;

	dijkstra(src, distances, parents);

	return distances;
}

vector<cost_type> light_graph::price_to_dst(int dst)
{
	vector<int> parents;
	vector<cost_type> distances;

	dijkstra(dst, distances, parents, -1, true);

	return distances;
}

bool light_graph::dijkstra(int from, std::vector<cost_type>& distances, std::vector<int>& parents, int to, bool reversed)
{
	using cipair = std::pair<cost_type, int>;

	vector<map<int, light_edge&>>& Ec = reversed ? Er : E;

	parents = std::move(vector<int>(V, -1));
	distances = std::move(vector<cost_type>(V, numeric_limits<double>::infinity()));
	vector<bool> closed(V, false);

	std::priority_queue<cipair, vector<cipair>, std::greater<cipair>> queue;

	// First node
	queue.push(make_pair(0, from));
	distances[from] = 0;
	parents[from] = from;

	// Until queue is empty
	while (!queue.empty()) {
		int src = queue.top().second;

		if (!temp_enabled_V[src])
			throw logic_error("Light graph error: queued node must be enabled");

		// Break if destination reached
		if (src == to)
			break;

		queue.pop();

		// Already closed
		if (closed[src])
			continue;

		// Loop for each edge from curr
		cost_type curr_dist = distances[src];
		for (auto it = Ec[src].begin(); it != Ec[src].end(); ++it) {
			// Skip disabled node
			if (!temp_enabled_V[it->first])
				continue;

			light_edge& edge = it->second;

			// Skip disabled edge
			if (!edge.enabled || !edge.temp_enabled)
				continue;

			int dst = it->first;
			cost_type new_dist = curr_dist + edge.cost + edge.toll;

			// New distance is better
			if (new_dist < distances[dst]) {
				distances[dst] = new_dist;
				parents[dst] = src;

				// Add a new entry to the queue
				queue.push(make_pair(new_dist, dst));
			}
		}

		// Close the vertex
		closed[src] = true;
	}

	// If there is no destination, always return true
	// If the queue is empty, the destination was not reached, return false
	return to < 0 || !queue.empty();
}
