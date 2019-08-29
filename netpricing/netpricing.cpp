// netpricing.cpp : Defines the entry point for the application.
//

#include "netpricing.h"
#include "problem.h"

#include <iostream>
#include <boost/graph/graph_traits.hpp>

using namespace std;
using namespace boost;

int main()
{
	cout << "Hello CMake." << endl;

	problem prob = problem::read_from_json("../../../../resources/problems/g10-1.json")[0];

	std::cout << "edges(g) = ";
	property_map<problem::graph_type, edge_tolled_t>::type edge_tolled_map = get(edge_tolled, prob.graph);
	graph_traits<problem::graph_type>::edge_iterator ei, ei_end;

	for (tie(ei, ei_end) = edges(prob.graph); ei != ei_end; ++ei) {
		cout << "(" << source(*ei, prob.graph) << "," << target(*ei, prob.graph) << ") " << edge_tolled_map[*ei] << endl;
	}

	prob.write_to_json("../../../../resources/problems/g10-1-copy.json");

	return 0;
}
