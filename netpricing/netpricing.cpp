// netpricing.cpp : Defines the entry point for the application.
//

#include "netpricing.h"
#include "problem.h"
#include "problem_generator.h"

#include <iostream>
#include <boost/graph/graph_traits.hpp>

using namespace std;
using namespace boost;

int main()
{
	cout << "Hello CMake." << endl;

	problem prob = random_problem(10, 20, 3, 0.5f);

	cout << "edges(g) = " << endl;
	property_map<problem::graph_type, edge_tolled_t>::type edge_tolled_map = get(edge_tolled, prob.graph);
	property_map<problem::graph_type, edge_weight_t>::type edge_weight_map = get(edge_weight, prob.graph);
	graph_traits<problem::graph_type>::edge_iterator ei, ei_end;

	for (tie(ei, ei_end) = edges(prob.graph); ei != ei_end; ++ei) {
		cout << "\t(" << source(*ei, prob.graph) << "," << target(*ei, prob.graph) << ") " << edge_tolled_map[*ei] << " " << edge_weight_map[*ei] << endl;
	}

	return 0;
}
