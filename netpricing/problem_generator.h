#pragma once

#include <algorithm>
#include <cmath>
#include <random>
#include <utility>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/erdos_renyi_generator.hpp>
#include <boost/graph/copy.hpp>
#include <boost/graph/grid_graph.hpp>

#include "problem.h"
#include "macros.h"
#include "graph_algorithm.h"
#include "shullpro/s_hull_pro.h"

// =================== GENERATOR COMPONENTS =====================

struct brotcorne_arc_cost_distribution {
	int lb, ub;
	float ub_prob;
	std::uniform_real_distribution<cost_type> cost_dist;
	std::uniform_real_distribution<float> prob_dist;

	brotcorne_arc_cost_distribution(int lb = 5, int ub = 35, float ub_prob = 0.2) :
		lb(lb), ub(ub), ub_prob(ub_prob), cost_dist(lb, ub), prob_dist(0, 1) { }

	template <class random_engine_type>
	cost_type operator()(random_engine_type& random_engine) {
		if (prob_dist(random_engine) < ub_prob)
			return ub;
		else
			return cost_dist(random_engine);
	}
};

static std::default_random_engine default_engine;
static brotcorne_arc_cost_distribution default_arc_cost_dist;

template<class random_engine_type, class demand_dist_type>
std::vector<commodity> random_commodities(int V, int K,
										  random_engine_type& random_engine,
										  demand_dist_type demand_dist) {
	std::uniform_int_distribution<int> vertex_dist(0, V - 1);
	std::uniform_int_distribution<int> vertex_dist_2(0, V - 2);

	// Commodities
	std::vector<commodity> commodities;

	LOOP(k, K) {
		int orig = vertex_dist(random_engine);
		int dest = vertex_dist_2(random_engine);
		if (dest >= orig)
			++dest;
		demand_type demand = demand_dist(random_engine);

		commodities.push_back(commodity{ orig, dest, demand });
	}

	return commodities;
}

template<class input_graph_type, class random_engine_type, class cost_dist_type>
problem::graph_type graph_with_random_costs(const input_graph_type& input_graph,
											random_engine_type& random_engine,
											cost_dist_type cost_dist) {
	using namespace boost;

	problem::graph_type graph(num_vertices(input_graph));

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

	return graph;
}

bool is_removable(problem::graph_type& graph,
				  problem::graph_type& toll_free_graph,
				  problem::edge_descriptor& edge_desc,
				  const std::vector<commodity>& commodities);

std::vector<problem::edge_descriptor> edge_order_by_occurrences(const std::vector<commodity>& commodities,
																const problem::graph_type& graph);

  // ======================== GENERATORS ==========================

template<class input_graph_type,
	class random_engine_type = std::default_random_engine,
	class cost_dist_type = brotcorne_arc_cost_distribution,
	class demand_dist_type = std::uniform_real_distribution<demand_type>>
	problem random_problem_from_graph(const input_graph_type& input_graph,
									  int num_commodities,
									  float toll_proportion,
									  random_engine_type& random_engine = default_engine,
									  cost_dist_type cost_dist = default_arc_cost_dist,
									  demand_dist_type demand_dist = demand_dist_type(1, 100))
{
	using namespace std;
	using namespace boost;

	// Commodities
	vector<commodity> commodities = random_commodities(num_vertices(input_graph), num_commodities, random_engine, demand_dist);

	// Building a graph with costs
	problem::graph_type graph = graph_with_random_costs(input_graph, random_engine, cost_dist);

	// Order the edges by occurrences in shortest paths (most frequent at back)
	vector<problem::edge_descriptor> candidates = edge_order_by_occurrences(commodities, graph);

	// Toll free graph to check reachability
	problem::graph_type toll_free_graph(graph);

	// Target number of tolled edges
	int target_tolled = round(toll_proportion * candidates.size());
	int num_tolled = 0;
	bool shuffled = false;

	auto tolled_map = get(edge_tolled, graph);
	auto cost_map = get(edge_weight, graph);

	while (num_tolled < target_tolled && candidates.size() > 0) {
		auto edge = candidates.back();
		candidates.pop_back();

		// If edge is removable, set it as tolled
		if (is_removable(graph, toll_free_graph, edge, commodities)) {
			tolled_map[edge] = true;
			cost_map[edge] /= 2;
			++num_tolled;
		}

		// If 2/3 target reached, shuffle the candidate set
		if (!shuffled && num_tolled * 3 >= target_tolled * 2) {
			shuffle(candidates.begin(), candidates.end(), random_engine);
			shuffled = true;
		}
	}

	return problem(graph, std::move(commodities));
}


template<class random_engine_type = std::default_random_engine>
boost::adjacency_list<> random_graph(int num_verts, int num_edges,
									 random_engine_type& random_engine = default_engine) {

	using graph_type = boost::adjacency_list<boost::setS>;
	using ergen = boost::erdos_renyi_iterator<random_engine_type, graph_type>;

	graph_type graph;
	bool is_connected = false;

	do {
		graph = graph_type(ergen(random_engine, num_verts, (long unsigned int)num_edges), ergen(), num_verts);
		is_connected = is_strongly_connected(graph);
	} while (!is_connected);

	boost::adjacency_list<> output_graph;
	boost::copy_graph(graph, output_graph);
	return output_graph;
}


template<class random_engine_type = std::default_random_engine,
	class cost_dist_type = brotcorne_arc_cost_distribution,
	class demand_dist_type = std::uniform_real_distribution<demand_type>>
	problem random_problem(int num_verts, int num_edges, int num_commodities,
						   float toll_proportion,
						   random_engine_type& random_engine = default_engine,
						   cost_dist_type cost_dist = default_arc_cost_dist,
						   demand_dist_type demand_dist = demand_dist_type(1, 100)) {

	auto graph = random_graph(num_verts, num_edges, random_engine);
	return random_problem_from_graph(graph, num_commodities, toll_proportion, random_engine, cost_dist, demand_dist);
}

// ====================== SPECIAL GRAPHS ========================

#define RANDOM_PROBLEM(name, ...) \
template<class random_engine_type = std::default_random_engine, \
	class cost_dist_type = brotcorne_arc_cost_distribution, \
	class demand_dist_type = std::uniform_real_distribution<demand_type>> \
	problem name(__VA_ARGS__, \
				int num_commodities, \
				float toll_proportion, \
				random_engine_type& random_engine = default_engine, \
				cost_dist_type cost_dist = default_arc_cost_dist, \
				demand_dist_type demand_dist = demand_dist_type(1, 100))

boost::adjacency_list<> grid_graph(int width, int height);

RANDOM_PROBLEM(random_grid_problem, int width, int height) {
	auto graph = grid_graph(width, height);
	return random_problem_from_graph(graph, num_commodities, toll_proportion, random_engine, cost_dist, demand_dist);
}

bool pointSortPredicate(const Shx& a, const Shx& b);

template<class random_engine_type>
std::vector<Triad> delaunay_triads(int num_points,
								   random_engine_type& random_engine) {
	// Points generation
	std::vector<Shx> pts;
	Shx pt;
	std::uniform_real_distribution<float> dist;

	for (int v = 0; v < num_points; v++) {
		pt.id = v;
		pt.r = dist(random_engine);
		pt.c = dist(random_engine);

		pts.push_back(pt);
	}

	std::sort(pts.begin(), pts.end(), pointSortPredicate);

	// S-hull
	std::vector<Triad> triads;
	s_hull_pro(pts, triads);

	return triads;
}

template<class random_engine_type>
boost::adjacency_list<> delaunay_graph(int num_points,
									   random_engine_type& random_engine) {
	std::vector<Triad> triads = delaunay_triads(num_points, random_engine);

	// Build the graph
	boost::adjacency_list<boost::setS> graph(num_points);
	for (const Triad& triad : triads) {
		// A-B
		boost::add_edge(triad.a, triad.b, graph);
		boost::add_edge(triad.b, triad.a, graph);
		// B-C
		boost::add_edge(triad.b, triad.c, graph);
		boost::add_edge(triad.c, triad.b, graph);
		// C-A
		boost::add_edge(triad.c, triad.a, graph);
		boost::add_edge(triad.a, triad.c, graph);
	}

	boost::adjacency_list<> output_graph;
	boost::copy_graph(graph, output_graph);
	return output_graph;
}

template<class random_engine_type>
boost::adjacency_list<> voronoi_graph(int num_seeds,
									  random_engine_type& random_engine) {
	std::vector<Triad> triads = delaunay_triads(num_seeds, random_engine);
	int num_triads = triads.size();

	// Build the graph
	boost::adjacency_list<boost::setS> graph(num_triads);
	LOOP(i, num_triads) {
		const Triad& triad = triads[i];
		// A-B
		int j = triad.ab >= 0 ? triad.ab : boost::add_vertex(graph);
		boost::add_edge(i, j, graph);
		boost::add_edge(j, i, graph);

		// B-C
		j = triad.bc >= 0 ? triad.bc : boost::add_vertex(graph);
		boost::add_edge(i, j, graph);
		boost::add_edge(j, i, graph);

		// C-A
		j = triad.ac >= 0 ? triad.ac : boost::add_vertex(graph);
		boost::add_edge(i, j, graph);
		boost::add_edge(j, i, graph);
	}

	boost::adjacency_list<> output_graph;
	boost::copy_graph(graph, output_graph);
	return output_graph;
}

RANDOM_PROBLEM(random_delaunay_problem, int num_points) {
	auto graph = delaunay_graph(num_points, random_engine);
	return random_problem_from_graph(graph, num_commodities, toll_proportion, random_engine, cost_dist, demand_dist);
}

RANDOM_PROBLEM(random_voronoi_problem, int num_seeds) {
	auto graph = voronoi_graph(num_seeds, random_engine);
	return random_problem_from_graph(graph, num_commodities, toll_proportion, random_engine, cost_dist, demand_dist);
}
