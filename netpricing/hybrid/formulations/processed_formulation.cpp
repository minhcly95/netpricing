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
	info = preproc->preprocess(*prob, k);
	V = *(info.V.rbegin()) + 1;
	A = *(info.A.rbegin()) + 1;
	A1 = *(info.A1.rbegin()) + 1;
	A2 = *(info.A2.rbegin()) + 1;

	lgraph = new light_graph(info.build_graph());
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
