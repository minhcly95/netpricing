#include "path_based_formulation.h"

#include "../../macros.h"
#include "../../graph/light_graph.h"

using namespace std;

path_based_formulation::path_based_formulation(const std::vector<path>& paths) :
	paths(paths), P(paths.size())
{
}

path_based_formulation::~path_based_formulation()
{
}

void path_based_formulation::prepare()
{
	light_graph lgraph(prob->graph);

	null_costs.resize(P);
	LOOP(p, P) null_costs[p] = lgraph.get_path_cost(paths[p], false);

	toll_sets.resize(P);
	LOOP(p, P) {
		auto toll_arcs = lgraph.get_toll_list(paths[p]);
		for (const auto& pair : toll_arcs) {
			auto edge = EDGE_FROM_SRC_DST(*prob, pair.first, pair.second);
			toll_sets[p].insert(EDGE_TO_A1(*prob, edge));
		}
	}
}