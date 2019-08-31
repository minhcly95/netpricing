#pragma once

#include <algorithm>
#include <random>
#include <utility>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/erdos_renyi_generator.hpp>

#include "problem.h"
#include "graph_algorithm.h"

static std::default_random_engine default_engine;

template<class input_graph_type,
	class random_engine_type = std::default_random_engine,
	class cost_dist_type = std::uniform_real_distribution<cost_type>,
	class demand_dist_type = std::uniform_real_distribution<demand_type>>
problem random_problem_from_graph(const input_graph_type& input_graph,
								 int num_commodities,
								 float toll_proportion,
								 random_engine_type& random_engine = default_engine,
								 cost_dist_type cost_dist = cost_dist_type(2, 20),
								 demand_dist_type demand_dist = demand_dist_type(1, 100))
{
	using namespace std;
	using namespace boost;

	// Number of vertices
	int num_verts = num_vertices(input_graph);

	uniform_int_distribution<int> vertex_dist(0, num_verts - 1);
	uniform_int_distribution<int> vertex_dist_2(0, num_verts - 2);

	// Commodities
	vector<commodity> commodities;

	for (int i = 0; i < num_commodities; ++i) {
		int orig = vertex_dist(random_engine);
		int dest = vertex_dist_2(random_engine);
		if (dest >= orig)
			++dest;
		demand_type demand = demand_dist(random_engine);

		commodities.push_back(commodity{ orig, dest, demand });
	}

	// Building a new graph
	problem::graph_type graph(num_verts);

	typename graph_traits<input_graph_type>::edge_iterator iei, iei_end;
	for (tie(iei, iei_end) = edges(input_graph); iei != iei_end; ++iei) {
		int src = source(*iei, input_graph);
		int dst = target(*iei, input_graph);
		cost_type cost = cost_dist(random_engine);

		problem::edge_property_type prop;
		get_property_value(prop, edge_weight) = cost;
		get_property_value(prop, edge_tolled) = false;

		add_edge(src, dst, prop, graph);
	}

	// Toll free graph to check reachability
	problem::graph_type toll_free_graph(graph);

	using edge_type = typename graph_traits<problem::graph_type>::edge_descriptor;
	typename graph_traits<problem::graph_type>::edge_iterator ei, ei_end;
	tie(ei, ei_end) = edges(graph);

	// All unchecked edges
	vector<edge_type> candidates(ei, ei_end);
	shuffle(candidates.begin(), candidates.end(), random_engine);

	// Target number of tolled edges
	int target_tolled = round(toll_proportion * candidates.size());
	int num_tolled = 0;

	auto tolled_map = get(edge_tolled, graph);
	auto cost_map = get(edge_weight, graph);

	while (num_tolled < target_tolled && candidates.size() > 0) {
		edge_type edge = candidates.back();
		candidates.pop_back();

		int src = source(edge, graph);
		int dst = target(edge, graph);

		// Try to remove edge and test for reachability
		remove_edge(src, dst, toll_free_graph);

		bool ok_to_remove = all_of(commodities.begin(), commodities.end(), [&toll_free_graph](const commodity& c) {
			int orig = c.origin;
			int dest = c.destination;
			return is_reachable(orig, dest, toll_free_graph);
								   });

		// If OK, set edge as tolled
		if (ok_to_remove) {
			tolled_map[edge] = true;
			cost_map[edge] /= 2;
			++num_tolled;
		}
		// Otherwise, re-add the removed edge
		else {
			add_edge(src, dst, toll_free_graph);
		}
	}

	return problem(graph, std::move(commodities));
}


template<class random_engine_type = std::default_random_engine>
boost::adjacency_list<> random_graph(int num_verts, int num_edges,
								 random_engine_type& random_engine = default_engine) {

	using graph_type = boost::adjacency_list<>;
	using ergen = boost::erdos_renyi_iterator<random_engine_type, graph_type>;

	graph_type graph;
	bool is_connected = false;

	do {
		graph = graph_type(ergen(random_engine, num_verts, (long unsigned int)num_edges), ergen(), num_verts);
		is_connected = is_strongly_connected(graph);
	} while (!is_connected);

	return graph;
}


template<class random_engine_type = std::default_random_engine,
	class cost_dist_type = std::uniform_real_distribution<cost_type>,
	class demand_dist_type = std::uniform_real_distribution<demand_type>>
problem random_problem(int num_verts, int num_edges, int num_commodities,
					   float toll_proportion,
					   random_engine_type& random_engine = default_engine,
					   cost_dist_type cost_dist = cost_dist_type(2, 20),
					   demand_dist_type demand_dist = demand_dist_type(1, 100)) {

	auto graph = random_graph(num_verts, num_edges, random_engine);
	return random_problem_from_graph(graph, num_commodities, toll_proportion, random_engine, cost_dist, demand_dist);
}

