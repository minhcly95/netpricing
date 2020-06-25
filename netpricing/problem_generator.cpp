#include "problem_generator.h"

#include "macros.h"

#include <iostream>
#include <boost/graph/dijkstra_shortest_paths.hpp>

using namespace std;
using namespace boost;

vector<problem::edge_descriptor> edge_order_by_occurrences(const vector<commodity>& commodities, const problem::graph_type& graph)
{
	int V = num_vertices(graph);
	int K = commodities.size();

	// Occurences of each edge in the shortest path of each commodity
	map<problem::edge_descriptor, int> occurrences;

	LOOP(k, K) {
		// Find the shortest path
		vector<int> parents(V);
		auto index_map = get(vertex_index, graph);

		boost::dijkstra_shortest_paths(graph, commodities[k].origin,
									   predecessor_map(make_iterator_property_map(parents.begin(), index_map)));

		// Count the occurences
		int current = commodities[k].destination;
		while (current != commodities[k].origin) {
			int prev = parents[current];
			auto edge = boost::edge(prev, current, graph).first;

			++occurrences[edge];

			current = prev;
		}
	}

	// Sort the edges
	auto edge_it = edges(graph);
	vector<problem::edge_descriptor> edge_order(edge_it.first, edge_it.second);

	sort(edge_order.begin(), edge_order.end(),
		 [&](auto& a, auto& b) { return occurrences[a] < occurrences[b]; });

	return edge_order;
}

vector<problem::edge_descriptor> edge_symmetric_order_by_occurrences(const vector<commodity>& commodities, const problem::graph_type& graph)
{
	int V = num_vertices(graph);
	int K = commodities.size();

	// Occurences of each edge in the shortest path of each commodity
	map<problem::edge_descriptor, int> occurrences;

	LOOP(k, K) {
		// Find the shortest path
		vector<int> parents(V);
		auto index_map = get(vertex_index, graph);

		boost::dijkstra_shortest_paths(graph, commodities[k].origin,
									   predecessor_map(make_iterator_property_map(parents.begin(), index_map)));

		// Count the occurences
		int current = commodities[k].destination;
		while (current != commodities[k].origin) {
			int prev = parents[current];
			int a = min(prev, current);
			int b = max(prev, current);
			auto edge = boost::edge(a, b, graph).first;

			++occurrences[edge];

			current = prev;
		}
	}

	// Sort the edges
	auto edge_it = edges(graph);
	vector<problem::edge_descriptor> edge_order(edge_it.first, edge_it.second);

	// Remove symmetric edges
	edge_order.erase(remove_if(edge_order.begin(), edge_order.end(),
							   [&](auto& e) {
								   return source(e, graph) > target(e, graph);
							   }),
					 edge_order.end());

	sort(edge_order.begin(), edge_order.end(),
		 [&](auto& a, auto& b) { return occurrences[a] < occurrences[b]; });

	return edge_order;
}

bool is_removable(problem::graph_type& graph, problem::graph_type& toll_free_graph, problem::edge_descriptor& edge_desc, const vector<commodity>& commodities)
{
	int src = source(edge_desc, graph);
	int dst = target(edge_desc, graph);

	// Try to remove edge and test for reachability
	remove_edge(src, dst, toll_free_graph);

	bool ok_to_remove = all_of(commodities.begin(), commodities.end(), [&toll_free_graph](const commodity& c) {
		return is_reachable(c.origin, c.destination, toll_free_graph);
							   });

	// Otherwise, re-add the removed edge
	if (!ok_to_remove) {
		add_edge(src, dst, toll_free_graph);
	}

	return ok_to_remove;
}

bool is_symmetrically_removable(problem::graph_type& graph, problem::graph_type& toll_free_graph, problem::edge_descriptor& edge_desc, const vector<commodity>& commodities)
{
	int src = source(edge_desc, graph);
	int dst = target(edge_desc, graph);

	// Try to remove edge and test for reachability
	remove_edge(src, dst, toll_free_graph);
	remove_edge(dst, src, toll_free_graph);

	bool ok_to_remove = all_of(commodities.begin(), commodities.end(), [&toll_free_graph](const commodity& c) {
		return is_reachable(c.origin, c.destination, toll_free_graph);
							   });

	// Otherwise, re-add the removed edge
	if (!ok_to_remove) {
		add_edge(src, dst, toll_free_graph);
		add_edge(dst, src, toll_free_graph);
	}

	return ok_to_remove;
}

boost::adjacency_list<> grid_graph(int width, int height) {
	using graph_type = boost::adjacency_list<>;

	graph_type graph(width * height);

	for (int i = 0; i < height; ++i) {
		for (int j = 0; j < width; ++j) {
			// Horizontal edges
			if (j + 1 < width) {
				boost::add_edge(i * width + j, i * width + j + 1, graph);
				boost::add_edge(i * width + j + 1, i * width + j, graph);
			}
			// Vertical edges
			if (i + 1 < height) {
				boost::add_edge(i * width + j, (i + 1) * width + j, graph);
				boost::add_edge((i + 1) * width + j, i * width + j, graph);
			}
		}
	}

	return graph;
}

bool pointSortPredicate(const Shx& a, const Shx& b)
{
	if (a.r < b.r)
		return true;
	else if (a.r > b.r)
		return false;
	else if (a.c < b.c)
		return true;
	else
		return false;
};
