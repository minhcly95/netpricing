#include "light_graph.h"

#include "../macros.h"

#include <algorithm>
#include <queue>
#include <tuple>
#include <set>

using namespace std;

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

vector<light_graph::path> light_graph::k_shortest_paths(int from, int to, int K, bool toll_free_break)
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

vector<light_graph::path> light_graph::toll_unique_paths(int from, int to, int k)
{
	vector<path> kpaths = k_shortest_paths(from, to, k, true);
	vector<path> rpaths;

	using odpair = pair<int, int>;
	set<set<odpair>> visited_sets;

	for (const path& path : kpaths) {
		// Get the set of toll arcs
		set<odpair> toll_set;
		for (int i = 0; i < path.size() - 1; i++)
			if (edge(path[i], path[i + 1]).is_tolled)
				toll_set.emplace(path[i], path[i + 1]);

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

std::vector<cost_type> light_graph::price_from_src(int src)
{
	vector<int> parents;
	vector<cost_type> distances;

	dijkstra(src, distances, parents);

	return distances;
}

std::vector<cost_type> light_graph::price_to_dst(int dst)
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
