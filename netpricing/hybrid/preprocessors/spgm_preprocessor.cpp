#include "spgm_preprocessor.h"

#include <iostream>

#include "../../problem.h"
#include "../../macros.h"

using namespace std;

// Helper functions
void add_toll_free_arc(preprocess_info& info, int& a, int src, int dst, cost_type cost) {
	using arc_bimap_rel = preprocess_info::arc_bimap::value_type;

	int a2 = info.A2.size();

	// If there exists a tolled arc with the same (src, dst), create a virtual node
	if (info.bimap_A.right.count(make_pair(src, dst))) {
		int virt = info.V.size();
		info.V.insert(virt);

		info.A.insert(a);
		info.A2.insert(a2);

		info.bimap_A.insert(arc_bimap_rel(a, make_pair(virt, dst)));
		info.bimap_A2.insert(arc_bimap_rel(a2, make_pair(virt, dst)));

		info.cost_A.emplace(a, 0);
		info.cost_A2.emplace(a2, 0);

		info.is_tolled.emplace(a, false);

		a++;
		a2++;
		dst = virt;
	}

	info.A.insert(a);
	info.A2.insert(a2);

	info.bimap_A.insert(arc_bimap_rel(a, make_pair(src, dst)));
	info.bimap_A2.insert(arc_bimap_rel(a2, make_pair(src, dst)));

	info.cost_A.emplace(a, cost);
	info.cost_A2.emplace(a2, cost);

	info.is_tolled.emplace(a, false);

	a++;
}

preprocess_info spgm_preprocessor::preprocess_impl(const light_graph& graph, const commodity& comm, int k)
{
	using arc_bimap_rel = preprocess_info::arc_bimap::value_type;

	light_graph original = preprocessor::preprocess_impl(graph, comm, k).build_graph();

	// Origin and destination prices
	int orig = comm.origin;
	int dest = comm.destination;
	original.set_toll_arcs_enabled(true);
	vector<cost_type> lower_orig = original.price_from_src(orig);
	vector<cost_type> lower_dest = original.price_to_dst(dest);

	original.set_toll_arcs_enabled(false);
	vector<cost_type> upper_orig = original.price_from_src(orig);
	vector<cost_type> upper_dest = original.price_to_dst(dest);

	cost_type lower_total = lower_orig[dest];
	cost_type upper_total = upper_orig[dest];

	// Info
	preprocess_info info;

	// List of tolled src, dst
	map<int, set<int>> all_src;
	map<int, set<int>> all_dst;

	LOOP(i, graph.V) info.V.insert(i);

	// Add all tolled arcs
	int a = 0;
	for (const light_edge& e : original.Eall) {
		if (!e.is_tolled) continue;

		cost_type cost = e.cost;
		int i = e.src, j = e.dst;

		// Elimination rules
		// Rules #1, #2: lower and upper prices are the same => arc has no effect
		if (lower_orig[j] >= upper_orig[j]) continue;
		if (lower_dest[i] >= upper_dest[i]) continue;

		// O-D (rule #5), I-D, O-J (enhanced rules #1, #2)
		if (upper_total <= lower_orig[i] + cost + lower_dest[j]) continue;
		if (upper_dest[i] <= cost + lower_dest[j]) continue;
		if (upper_orig[j] <= lower_orig[i] + cost) continue;

		// Pass, add to info
		info.A.insert(a);
		info.A1.insert(e.index);

		info.bimap_A.insert(arc_bimap_rel(a, make_pair(i, j)));
		info.bimap_A1.insert(arc_bimap_rel(e.index, make_pair(i, j)));

		info.cost_A.emplace(a, cost);
		info.cost_A1.emplace(e.index, cost);

		info.is_tolled.emplace(a, true);

		all_src[i].insert(j);
		all_dst[j].insert(i);

		a++;
	}

	// Exclude O, D from set of src, dst
	all_src.erase(orig);
	all_src.erase(dest);
	all_dst.erase(orig);
	all_dst.erase(dest);

	// Add toll-free arcs
	// O-D arc
	add_toll_free_arc(info, a, orig, dest, upper_total);

	// O-src arcs
	for (auto& pair : all_src) {
		int i = pair.first;
		cost_type cost = upper_orig[i];
		if (!isfinite(cost)) continue;

		// Rule #7
		if (upper_total <= cost + lower_dest[i]) continue;

		add_toll_free_arc(info, a, orig, i, cost);
	}

	// dst-D arcs
	for (auto& pair : all_dst) {
		int j = pair.first;
		cost_type cost = upper_dest[j];
		if (!isfinite(cost)) continue;

		// Rule #8
		if (upper_total <= lower_orig[j] + cost) continue;

		add_toll_free_arc(info, a, j, dest, cost);
	}

	// dst-src arcs
	for (auto& pair_j : all_dst) {
		int j = pair_j.first;
		vector<cost_type> upper_j = original.price_from_src(j);

		for (auto& pair_i : all_src) {
			int i = pair_i.first;
			cost_type cost = upper_j[i];

			// Self-loop or disconnect pair
			if (j == i || !isfinite(cost)) continue;

			// Rules #1, #2
			if (lower_orig[i] >= upper_orig[i]) continue;
			if (lower_dest[j] >= upper_dest[j]) continue;

			// O-D (rule #6), J-D (rule #3), O-I (rule #4)
			if (upper_total <= lower_orig[j] + cost + lower_dest[i]) continue;
			if (upper_dest[j] <= cost + lower_dest[i]) continue;
			if (upper_orig[i] <= lower_orig[j] + cost) continue;

			// Pass, add to info
			if (pair_j.second.count(i)) {
				// i connects to j by a tolled arc
				// only connect j to i if j has another i' and i has another j'
				if (pair_j.second.size() > 1 && pair_i.second.size() > 1)
					add_toll_free_arc(info, a, j, i, cost);
			}
			else {
				// i does not connect to j
				add_toll_free_arc(info, a, j, i, cost);
			}
		}
	}

	info.prune(orig, dest);
	info.clean();

	cout << "Comm " << k << ": " << info.V.size() << " nodes, "
		<< info.A.size() << " arcs (" << info.A1.size() << " tolled)" << endl;

	return info;
}
