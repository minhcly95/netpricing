#include "sstd_formulation.h"

#include "../../macros.h"

using namespace std;

sstd_formulation::sstd_formulation(const vector<path>& paths, const light_graph& original) :
	general_formulation(paths, original, ARC, ARC, STRONG_DUAL, DIRECT), spgm_preproc()
{
}

void sstd_formulation::prepare()
{
	// Normal preprocessing
	info = preproc.preprocess_impl(original, prob->commodities[k], k);
	lgraph = new light_graph(info.build_graph());

	// SPGM preprocessing
	preprocess_info info_spgm = spgm_preproc.preprocess_impl(*lgraph, prob->commodities[k], k);
	if (info_spgm.A.size() < info.A.size()) {
		info = info_spgm;
		lgraph = new light_graph(info_spgm.build_graph());
		cout << "Comm " << k << " (SSTD): SPGM applied" << endl;
	}

	V = *(info.V.rbegin()) + 1;
	A = *(info.A.rbegin()) + 1;
	A1 = *(info.A1.rbegin()) + 1;
	A2 = *(info.A2.rbegin()) + 1;

	// Process path
	toll_sets.resize(P);
	arc_sets.resize(P);

	LOOP(p, P) {
		// Extract toll sets
		auto toll_arcs = original.get_toll_list(paths[p]);
		for (const auto& pair : toll_arcs) {
			auto edge = EDGE_FROM_SRC_DST(*prob, pair.first, pair.second);
			toll_sets[p].insert(EDGE_TO_A1(*prob, edge));
		}

		// Transform path
		paths[p] = convert_path_to_new_graph(paths[p], original, *lgraph);

		// Extract arc sets
		for (int i = 0; i < paths[p].size() - 1; i++) {
			int a = info.bimap_A.right.at(make_pair(paths[p][i], paths[p][i + 1]));
			arc_sets[p].insert(a);
		}
	}

	null_costs.resize(P);
	LOOP(p, P) null_costs[p] = lgraph->get_path_cost(paths[p], false);
}
