#include "light_graph.h"

#include "../macros.h"

using namespace std;

light_graph::light_graph(const problem_base::graph_type& graph) :
	V(boost::num_vertices(graph)), E(V)
{
	auto cost_map = boost::get(boost::edge_weight, graph);

	problem_base::edge_iterator ei, ei_end;
	for (tie(ei, ei_end) = boost::edges(graph); ei != ei_end; ++ei) {
		int src = boost::source(*ei, graph);
		int dst = boost::target(*ei, graph);
		cost_type cost = cost_map[*ei];
		E[src][dst] = light_edge{
			.src = src,
			.dst = dst,
			.cost = cost,
			.toll = 0,
			.enabled = true
		};
	}
}

light_graph::path light_graph::shortest_path(int from, int to)
{
	vector<int> parents(V, -1);
	vector<cost_type> distances(V, numeric_limits<double>::infinity());
	vector<bool> closed(V, false);

	std::priority_queue<cipair, vector<cipair>, std::greater<cipair>> queue;

	// First node
	queue.push(make_pair(0, from));
	distances[from] = 0;
	parents[from] = from;

	// Until queue is empty
	while (!queue.empty()) {
		int src = queue.top().second;
		queue.pop();

		// Break if destination reached
		if (src == to)
			break;

		// Already closed
		if (closed[src])
			continue;

		// Loop for each edge from curr
		cost_type curr_dist = distances[src];
		for (auto it = E[src].begin(); it != E[src].end(); ++it) {
			light_edge& edge = it->second;

			// Skip disabled edge
			if (!edge.enabled)
				continue;

			int dst = edge.dst;
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
