#include "problem.h"

#include <iostream>
#include <fstream>
#include <map>

#include <boost/graph/graph_traits.hpp>
#include <nlohmann/json.hpp>

using namespace std;
using namespace boost;
using json = nlohmann::json;

vector<problem> problem::read_from_json(std::string filename)
{
	vector<problem> all_problems;

	ifstream is(filename);
	json root;
	is >> root;

	for (const json& problem_obj : root) {
		// Read number of vertices
		int num_vertices = problem_obj["V"].get<int>();
		graph_type graph(num_vertices);

		// Read edges
		for (const json& edge : problem_obj["A"]) {
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
		vector<commodity> commodities(problem_obj["K"].size());
		for (const json& comm : problem_obj["K"]) {
			// Data from Json
			int index = comm["index"].get<int>() - 1;
			int orig = comm["orig"].get<int>() - 1;
			int dest = comm["dest"].get<int>() - 1;
			demand_type demand = comm["demand"].get<demand_type>();

			commodities[index] = commodity{ orig, dest, demand };
		}

		// Add the problem into the list
		all_problems.push_back(problem{ graph, commodities });
	}

	return all_problems;
}

void problem::write_to_json(std::string filename, const std::vector<problem>& problems)
{
	json root;

	for (const problem& problem : problems) {
		// Problem node
		json problem_obj;

		// Number of vertices
		problem_obj["V"] = num_vertices(problem.graph);

		// Edges
		json edges_obj;

		auto cost_map = get(edge_weight, problem.graph);
		auto tolled_map = get(edge_tolled, problem.graph);
		graph_traits<problem::graph_type>::edge_iterator ei, ei_end;

		for (tie(ei, ei_end) = edges(problem.graph); ei != ei_end; ++ei) {
			// Edge data
			int src = source(*ei, problem.graph);
			int dst = target(*ei, problem.graph);
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

		problem_obj["A"] = edges_obj;

		// Commodities
		json commodities_obj;

		for (int i = 0; i < problem.commodities.size(); ++i) {
			const commodity& c = problem.commodities[i];

			json commodity_obj;
			commodity_obj["index"] = i + 1;
			commodity_obj["orig"] = c.origin + 1;
			commodity_obj["dest"] = c.destination + 1;
			commodity_obj["demand"] = c.demand;

			commodities_obj.push_back(commodity_obj);
		}

		problem_obj["K"] = commodities_obj;

		// Add to root node
		root.push_back(problem_obj);
	}

	ofstream os(filename);
	os << root;
}

void problem::write_to_json(std::string filename) const
{
	write_to_json(filename, vector<problem> { *this });
}
