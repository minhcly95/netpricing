#include "problem_multi.h"

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

problem_multi::problem_multi(const vector<graph_type>& _graphs, const vector<commodity>& commodities) :
	graphs(_graphs), commodities(commodities) {
	update();
}

problem_multi::problem_multi(vector<graph_type>&& _graphs, std::vector<commodity>&& commodities) :
	graphs(std::move(_graphs)), commodities(std::move(commodities))
{
	update();
}

problem_multi::problem_multi(const problem_multi& prob) : problem_multi(prob.graphs, prob.commodities) { }

problem_multi::problem_multi(problem_multi&& prob) :
	graphs(std::move(prob.graphs)), commodities(std::move(prob.commodities)),
	big_m(std::move(prob.big_m)),
	big_n(std::move(prob.big_n))
{
	update_graph_data();
	update_indices();
}

problem_multi::problem_multi(const problem& prob) :
	graphs(prob.commodities.size(), prob.graph), commodities(prob.commodities)
{
	update();
}

problem_multi problem_multi::parse_json(const json& json_obj) {
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

	// Read number of vertices
	int num_vertices = json_obj["V"].get<int>();
	vector<graph_type> graphs;
	LOOP(k, commodities.size())
		graphs.push_back(graph_type(num_vertices));

	// Read arcs
	LOOP (k, commodities.size()) {
		for (const json& edge : json_obj["A"][k]) {
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
			add_edge(src, dst, prop, graphs[k]);
		}
	}


	// Add the problem into the list
	return problem_multi(std::move(graphs), std::move(commodities));
}

json problem_multi::get_json(const problem_multi& prob) {
	json json_obj;

	// Number of vertices
	json_obj["V"] = num_vertices(prob.graphs[0]);

	// Edges
	json all_edges_obj;

	LOOP(k, prob.commodities.size()) {
		json edges_obj;

		auto cost_map = get(edge_weight, prob.graphs[k]);
		auto tolled_map = get(edge_tolled, prob.graphs[k]);
		graph_traits<problem_multi::graph_type>::edge_iterator ei, ei_end;

		for (tie(ei, ei_end) = edges(prob.graphs[k]); ei != ei_end; ++ei) {
			// Edge data
			int src = source(*ei, prob.graphs[k]);
			int dst = target(*ei, prob.graphs[k]);
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

		all_edges_obj.push_back(edges_obj);
	}

	json_obj["A"] = all_edges_obj;

	// Commodities
	json commodities_obj;

	LOOP(k, prob.commodities.size()) {
		const commodity& c = prob.commodities[k];

		json commodity_obj;
		commodity_obj["index"] = k + 1;
		commodity_obj["orig"] = c.origin + 1;
		commodity_obj["dest"] = c.destination + 1;
		commodity_obj["demand"] = c.demand;

		commodities_obj.push_back(commodity_obj);
	}

	json_obj["K"] = commodities_obj;

	// Add to root node
	return json_obj;
}

json problem_multi::get_json() const {
	return get_json(*this);
}

void problem_multi::update()
{
	update_graph_data();
	update_indices();
	update_big_mn();
}

void problem_multi::update_graph_data() {
	LOOP(k, commodities.size()) {
		is_tolled_maps.push_back(boost::get(edge_tolled, graphs[k]));
		cost_maps.push_back(boost::get(boost::edge_weight, graphs[k]));
		tollfree_graphs.push_back(tollfree_graph_type(graphs[k], edge_tollfree_predicate<edge_tolled_map_type>(is_tolled_maps[k])));
	}
}

void problem_multi::update_indices()
{
	using namespace std;
	using namespace boost;
	using edge_index_rel = edge_index_bimap_type::value_type;
	using src_dst_index_rel = src_dst_index_bimap_type::value_type;

	// Reset
	int K = commodities.size();
	tolled_index_common_map.clear();
	tolled_index_maps = vector<edge_index_bimap_type>(K);
	tollfree_index_maps = vector<edge_index_bimap_type>(K);
	alledges_index_maps = vector<edge_index_bimap_type>(K);

	// Common tolled arcs mapping
	int total_num_tolled = 0;
	max_edge_index = 0;

	// Mapping
	LOOP(k, commodities.size()) {
		edge_iterator ei, ei_end;
		int tollfree_index = 0;

		for (tie(ei, ei_end) = edges(graphs[k]); ei != ei_end; ++ei) {
			bool is_tolled = is_tolled_maps[k][*ei];
			if (is_tolled) {
				int src = source(*ei, graphs[k]);
				int dst = target(*ei, graphs[k]);
				auto src_dst = make_pair(src, dst);

				// New tolled arc
				if (tolled_index_common_map.right.find(src_dst) == tolled_index_common_map.right.end()) {
					tolled_index_common_map.insert(src_dst_index_rel(total_num_tolled++, src_dst));
				}

				tolled_index_maps[k].insert(edge_index_rel(tolled_index_common_map.right.at(src_dst), *ei));
			}
			else {
				tollfree_index_maps[k].insert(edge_index_rel(tollfree_index++, *ei));
			}
		}

		max_edge_index = max(max_edge_index, tollfree_index);
	}

	max_edge_index += total_num_tolled;

	// All edges mapping
	LOOP(k, commodities.size()) {
		auto alledges_inserter = inserter(alledges_index_maps[k], alledges_index_maps[k].begin());

		copy(tolled_index_maps[k].begin(), tolled_index_maps[k].end(), alledges_inserter);
		transform(tollfree_index_maps[k].begin(), tollfree_index_maps[k].end(), alledges_inserter,
				  [total_num_tolled](const edge_index_rel& entry) {
					  return edge_index_rel(entry.left + total_num_tolled, entry.right);
				  });
	}
}

void problem_multi::update_big_mn()
{
	int V = num_vertices(graphs[0]);
	int K = commodities.size();
	int A1 = tolled_index_common_map.size();

	big_m = cost_matrix(K, cost_array(A1));
	big_n = cost_array(A1);

	cost_matrix tollfree_o(K, cost_array(V)), nulltoll_o(K, cost_array(V));
	LOOP(k, K) {
		int o = commodities[k].origin;
		GET_DISTANCE_MAP(graphs[k], o, nulltoll_o[k]);
		GET_DISTANCE_MAP(tollfree_graphs[k], o, tollfree_o[k]);
	}

	LOOP(a, A1) {
		big_n[a] = 0;

		LOOP(k, K) {
			auto map_it = tolled_index_maps[k].left.find(a);
			if (map_it == tolled_index_maps[k].left.end()) {
				big_m[k][a] = 0;
				continue;
			}

			auto edge = map_it->get_right();
			int i = source(edge, graphs[k]);
			int j = target(edge, graphs[k]);
			cost_type c = cost_maps[k][edge];

			cost_array tollfree_i(V), nulltoll_j(V);
			GET_DISTANCE_MAP(graphs[k], j, nulltoll_j);
			GET_DISTANCE_MAP(tollfree_graphs[k], i, tollfree_i);

			int o = commodities[k].origin;
			int d = commodities[k].destination;

			cost_type m1, m2, m3, m4;
			m1 = tollfree_i[j] - c;
			m2 = tollfree_o[k][j] - nulltoll_o[k][i] - c;
			m3 = tollfree_i[d] - c - nulltoll_j[d];
			m4 = tollfree_o[k][d] - nulltoll_o[k][i] - c - nulltoll_j[d];

			big_m[k][a] = max((cost_type)0, min({ m1, m2, m3, m4 }));
			big_n[a] = max(big_n[a], big_m[k][a]);
		}
	}
}

cost_type problem_multi::get_obj_upper_bound() const
{
	int V = num_vertices(graphs[0]);
	int K = commodities.size();

	cost_type sum_obj = 0;
	LOOP(k, K) {
		int o = commodities[k].origin;
		int d = commodities[k].destination;

		cost_array tollfree(V), nulltoll(V);
		GET_DISTANCE_MAP(graphs[k], o, nulltoll);
		GET_DISTANCE_MAP(tollfree_graphs[k], o, tollfree);

		sum_obj += (tollfree[d] - nulltoll[d]) * commodities[k].demand;
	}

	return sum_obj;
}