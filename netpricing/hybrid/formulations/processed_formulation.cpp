#include "processed_formulation.h"

#include "../../macros.h"

using namespace std;

processed_formulation::processed_formulation()
{
	preproc = new preprocessor();	// Default preprocessor
}

processed_formulation::processed_formulation(preprocessor* _preproc) :
	preproc(_preproc)
{
}

processed_formulation::~processed_formulation()
{
	delete preproc;
	delete lgraph;
}

void processed_formulation::preprocess()
{
	info = preproc->preprocess(*prob);
	V = info.V.back() + 1;
	A = info.A.back() + 1;
	A1 = info.A1.back() + 1;
	A2 = info.A2.back() + 1;

	init_graph();
}

void processed_formulation::init_graph()
{
	lgraph = new light_graph(V);

	problem_base::edge_iterator ei, ei_end;
	LOOP_INFO(a, A) {
		auto arc = info.bimap_A.left.at(a);
		cost_type cost = info.cost_A.at(a);
		bool is_tolled = info.is_tolled.at(a);

		lgraph->Eall.emplace_back(light_edge{
			.src = arc.first,
			.dst = arc.second,
			.cost = cost,
			.is_tolled = is_tolled,
			.toll = 0,
			.enabled = true,
			.temp_enabled = true
								  });
	}

	for (auto& edge : lgraph->Eall) {
		lgraph->E[edge.src].emplace(edge.dst, edge);
		lgraph->Er[edge.dst].emplace(edge.src, edge);
	}
}

std::vector<int> processed_formulation::get_path(const NumArray& tvals)
{
	// Set toll
	LOOP_INFO(a, A1) {
		auto arc = info.bimap_A1.left.at(a);
		lgraph->edge(arc).toll = tvals[a] * TOLL_PREFERENCE;		// Prefer tolled arcs
	}

	// Solve
	return lgraph->shortest_path(prob->commodities[k].origin, prob->commodities[k].destination);
}
