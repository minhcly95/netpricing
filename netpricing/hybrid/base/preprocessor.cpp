#include "preprocessor.h"

#include <algorithm>

#include "../../problem.h"
#include "../../macros.h"

using namespace std;

int preprocess_info::a_to_a1(int a)
{
	return bimap_A1.right.at(bimap_A.left.at(a));
}

int preprocess_info::a_to_a2(int a)
{
	return bimap_A2.right.at(bimap_A.left.at(a));
}

int preprocess_info::a1_to_a(int a1)
{
	return bimap_A.right.at(bimap_A1.left.at(a1));
}

int preprocess_info::a2_to_a(int a2)
{
	return bimap_A.right.at(bimap_A2.left.at(a2));
}

int preprocess_info::src_dst_to_a(int src, int dst)
{
	return bimap_A.right.at(make_pair(src, dst));
}

int preprocess_info::src_dst_to_a1(int src, int dst)
{
	return bimap_A1.right.at(make_pair(src, dst));
}

int preprocess_info::src_dst_to_a2(int src, int dst)
{
	return bimap_A2.right.at(make_pair(src, dst));
}

// Default proprocessor: extract problem to preprocess_info
preprocess_info preprocessor::preprocess(const problem& prob)
{
	using arc_bimap_rel = preprocess_info::arc_bimap::value_type;

	preprocess_info info;

	LOOP(i, boost::num_vertices(prob.graph)) info.V.push_back(i);
	LOOP(a, boost::num_edges(prob.graph)) info.A.push_back(a);
	LOOP(a, prob.tolled_index_map.size()) info.A1.push_back(a);
	LOOP(a, prob.tollfree_index_map.size()) info.A2.push_back(a);

	for (int a : info.A) {
		SRC_DST_FROM_A(prob, a);
		cost_type cost = prob.cost_map[edge];
		bool is_tolled = prob.is_tolled_map[edge];

		info.bimap_A.insert(arc_bimap_rel(a, make_pair(src, dst)));
		info.cost_A.emplace(a, cost);
		info.is_tolled.emplace(a, is_tolled);

		if (is_tolled) {
			int a1 = A_TO_A1(prob, a);
			info.bimap_A1.insert(arc_bimap_rel(a1, make_pair(src, dst)));
			info.cost_A1.emplace(a1, cost);
		}
		else {
			int a2 = A_TO_A2(prob, a);
			info.bimap_A2.insert(arc_bimap_rel(a2, make_pair(src, dst)));
			info.cost_A2.emplace(a2, cost);
		}
	}

	return info;
}
