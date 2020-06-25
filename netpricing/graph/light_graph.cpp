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

light_graph::light_graph(int V) :
	V(V), Eall(), E(V), Er(V),
	temp_enabled_V(V, true)
{}

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

	LOOP(a, Eall.size()) {
		auto& edge = Eall[a];
		E[edge.src].emplace(edge.dst, a);
		Er[edge.dst].emplace(edge.src, a);
	}
}

light_edge& light_graph::edge(int src, int dst)
{
	return Eall[E[src].at(dst)];
}

light_edge& light_graph::edge(const iipair& pair)
{
	return Eall[E[pair.first].at(pair.second)];
}

const light_edge& light_graph::edge(int src, int dst) const
{
	return Eall[E[src].at(dst)];
}

const light_edge& light_graph::edge(const iipair& pair) const
{
	return Eall[E[pair.first].at(pair.second)];
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

cost_type light_graph::get_path_cost(const path& p, bool with_toll) const
{
	cost_type sum = 0;

	for (int i = 0; i < p.size() - 1; i++) {
		const auto& edge = this->edge(p[i], p[i + 1]);
		sum += edge.cost + (with_toll ? edge.toll : 0);
	}

	return sum;
}

cost_type light_graph::get_path_toll(const path& p) const
{
	cost_type sum = 0;

	for (int i = 0; i < p.size() - 1; i++) {
		const auto& edge = this->edge(p[i], p[i + 1]);
		if (edge.is_tolled)
			sum += edge.toll;
	}

	return sum;
}

toll_set light_graph::get_toll_set(const path& p) const
{
	toll_set toll_set;
	for (int i = 0; i < p.size() - 1; i++)
		if (edge(p[i], p[i + 1]).is_tolled)
			toll_set.emplace(p[i], p[i + 1]);
	return toll_set;
}

toll_list light_graph::get_toll_list(const path& p) const
{
	toll_list toll_list;
	for (int i = 0; i < p.size() - 1; i++)
		if (edge(p[i], p[i + 1]).is_tolled)
			toll_list.emplace_back(p[i], p[i + 1]);
	return toll_list;
}

bool light_graph::is_toll_free(const path& p) const
{
	for (int i = 0; i < p.size() - 1; i++)
		if (edge(p[i], p[i + 1]).is_tolled)
			return false;
	return true;
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

	vector<map<int, int>>& Ec = reversed ? Er : E;

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

			light_edge& edge = Eall[it->second];

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
