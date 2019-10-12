#include "problem.h"

#include <algorithm>
#include <iostream>
#include <fstream>
#include <functional>
#include <map>
#include <numeric>
#include <utility>

#include <boost/graph/copy.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include "macros.h"

#define GET_DISTANCE_MAP(graph, source, map) \
	boost::dijkstra_shortest_paths(graph, source, boost::distance_map(boost::make_iterator_property_map(map.begin(), boost::get(boost::vertex_index, graph))))

using namespace std;
using namespace boost;
using json = nlohmann::json;

problem::problem(const graph_type& _graph, const std::vector<commodity>& commodities) :
	graph(_graph), commodities(commodities),
	is_tolled_map(boost::get(edge_tolled, graph)),
	cost_map(boost::get(boost::edge_weight, graph)),
	tollfree_graph(graph, edge_tollfree_predicate<edge_tolled_map_type>(is_tolled_map))
{
	update();
}

problem::problem(const graph_type& _graph, std::vector<commodity>&& commodities) :
	graph(_graph), commodities(std::move(commodities)),
	is_tolled_map(boost::get(edge_tolled, graph)),
	cost_map(boost::get(boost::edge_weight, graph)),
	tollfree_graph(graph, edge_tollfree_predicate<edge_tolled_map_type>(is_tolled_map))
{
	update();
}

problem::problem(const problem& prob) : problem(prob.graph, prob.commodities) { }

problem::problem(problem&& prob) :
	graph(prob.graph), commodities(std::move(prob.commodities)),
	is_tolled_map(boost::get(edge_tolled, graph)),
	cost_map(boost::get(boost::edge_weight, graph)),
	tollfree_graph(graph, edge_tollfree_predicate<edge_tolled_map_type>(is_tolled_map)),
	big_m(std::move(prob.big_m)),
	big_n(std::move(prob.big_n))
{
	update_indices();
}

problem problem::parse_json(const json& json_obj) {
	// Read number of vertices
	int num_vertices = json_obj["V"].get<int>();
	graph_type graph(num_vertices);

	// Read edges
	for (const json& edge : json_obj["A"]) {
		// Data from Json
		int src = edge["src"].get<int>() - 1;
		int dst = edge["dst"].get<int>() - 1;
		cost_type cost = edge["cost"].get<cost_type>();
		bool tolled = edge["toll"].get<bool>();

		// Edge properties (cost, tolled)
		edge_property_type prop;
		get_property_value(prop, edge_weight) = cost;
		get_property_value(prop, edge_tolled) = tolled;

		// Add edge to graph
		add_edge(src, dst, prop, graph);
	}

	// Read commodities
	vector<commodity> commodities(json_obj["K"].size());
	for (const json& comm : json_obj["K"]) {
		// Data from Json
		int index = comm["index"].get<int>() - 1;
		int orig = comm["orig"].get<int>() - 1;
		int dest = comm["dest"].get<int>() - 1;
		demand_type demand = comm["demand"].get<demand_type>();

		commodities[index] = commodity{ orig, dest, demand };
	}

	// Add the problem into the list
	return problem(graph, std::move(commodities));
}

json problem::get_json(const problem& prob) {
	json json_obj;

	// Number of vertices
	json_obj["V"] = num_vertices(prob.graph);

	// Edges
	json edges_obj;

	auto cost_map = get(edge_weight, prob.graph);
	auto tolled_map = get(edge_tolled, prob.graph);
	graph_traits<problem::graph_type>::edge_iterator ei, ei_end;

	for (tie(ei, ei_end) = edges(prob.graph); ei != ei_end; ++ei) {
		// Edge data
		int src = source(*ei, prob.graph);
		int dst = target(*ei, prob.graph);
		cost_type cost = cost_map[*ei];
		bool tolled = tolled_map[*ei];

		// Edge ptree node
		json edge;
		edge["src"] = src + 1;
		edge["dst"] = dst + 1;
		edge["cost"] = cost;
		edge["toll"] = tolled;

		edges_obj.push_back(edge);
	}

	json_obj["A"] = edges_obj;

	// Commodities
	json commodities_obj;

	for (int i = 0; i < prob.commodities.size(); ++i) {
		const commodity& c = prob.commodities[i];

		json commodity_obj;
		commodity_obj["index"] = i + 1;
		commodity_obj["orig"] = c.origin + 1;
		commodity_obj["dest"] = c.destination + 1;
		commodity_obj["demand"] = c.demand;

		commodities_obj.push_back(commodity_obj);
	}

	json_obj["K"] = commodities_obj;

	// Add to root node
	return json_obj;
}

vector<problem> problem::read_from_json(std::string filename)
{
	vector<problem> all_problems;

	ifstream is(filename);
	json root;
	is >> root;

	for (const json& problem_obj : root) {
		// Add the problem into the list
		all_problems.push_back(std::move(problem::parse_json(problem_obj)));
	}

	return all_problems;
}

void problem::write_to_json(std::string filename, const std::vector<problem>& problems)
{
	json root;

	for (const problem& problem : problems) {
		// Add to root node
		root.push_back(std::move(problem::get_json(problem)));
	}

	ofstream os(filename);
	os << root;
}

json problem::get_json() const {
	return get_json(*this);
}

void problem::write_to_json(std::string filename) const
{
	write_to_json(filename, vector<problem> { *this });
}

void problem::update()
{
	update_indices();
	update_big_mn();
}

void problem::update_indices()
{
	using namespace std;
	using namespace boost;
	using edge_index_rel = edge_index_bimap_type::value_type;

	// Reset
	tolled_index_map.clear();
	tollfree_index_map.clear();
	alledges_index_map.clear();

	// Mapping
	edge_iterator ei, ei_end;
	int tolled_index = 0, tollfree_index = 0;

	for (tie(ei, ei_end) = edges(graph); ei != ei_end; ++ei) {
		bool is_tolled = is_tolled_map[*ei];
		if (is_tolled) {
			tolled_index_map.insert(edge_index_rel(tolled_index++, *ei));
		}
		else {
			tollfree_index_map.insert(edge_index_rel(tollfree_index++, *ei));
		}
	}

	// All edges mapping
	auto alledges_inserter = inserter(alledges_index_map, alledges_index_map.begin());

	copy(tolled_index_map.begin(), tolled_index_map.end(), alledges_inserter);
	transform(tollfree_index_map.begin(), tollfree_index_map.end(), alledges_inserter,
			  [tolled_index](const edge_index_rel& entry) {
				  return edge_index_rel(entry.left + tolled_index, entry.right);
			  });
}

void problem::update_big_mn()
{
	int V = num_vertices(graph);
	int K = commodities.size();
	int A1 = tolled_index_map.size();

	big_m = cost_matrix(K, cost_array(A1));
	big_n = cost_array(A1);

	cost_matrix tollfree_o(K, cost_array(V)), nulltoll_o(K, cost_array(V));
	LOOP(k, K) {
		int o = commodities[k].origin;
		GET_DISTANCE_MAP(graph, o, nulltoll_o[k]);
		GET_DISTANCE_MAP(tollfree_graph, o, tollfree_o[k]);
	}

	LOOP(a, A1) {
		auto edge = tolled_index_map.left.at(a);
		int i = source(edge, graph);
		int j = target(edge, graph);
		cost_type c = cost_map[edge];

		cost_array tollfree_i(V), nulltoll_j(V);
		GET_DISTANCE_MAP(graph, j, nulltoll_j);
		GET_DISTANCE_MAP(tollfree_graph, i, tollfree_i);

		big_n[a] = -numeric_limits<cost_type>::infinity();
		LOOP(k, K) {
			int o = commodities[k].origin;
			int d = commodities[k].destination;

			cost_type m1, m2, m3, m4;
			m1 = tollfree_i[j] - c;
			m2 = tollfree_o[k][j] - nulltoll_o[k][i] - c;
			m3 = tollfree_i[d] - c - nulltoll_j[d];
			m4 = tollfree_o[k][d] - nulltoll_o[k][i] - c - nulltoll_j[d];

			// M has 0.001f tolerance because exact M may cause infeasibility of subproblem
			big_m[k][a] = max((cost_type)0, min({ m1, m2, m3, m4 }));
			big_n[a] = max(big_n[a], big_m[k][a]);
		}
	}
}

cost_type problem::get_obj_upper_bound() const
{
	int V = num_vertices(graph);
	int K = commodities.size();

	cost_type sum_obj = 0;
	LOOP(k, K) {
		int o = commodities[k].origin;
		int d = commodities[k].destination;

		cost_array tollfree(V), nulltoll(V);
		GET_DISTANCE_MAP(graph, o, nulltoll);
		GET_DISTANCE_MAP(tollfree_graph, o, tollfree);

		sum_obj += (tollfree[d] - nulltoll[d]) * commodities[k].demand;
	}

	return sum_obj;
}