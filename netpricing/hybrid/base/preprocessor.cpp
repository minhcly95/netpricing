#include "preprocessor.h"

#include <algorithm>
#include <iostream>

#include "../../problem.h"
#include "../../macros.h"

using namespace std;

int preprocess_info::a_to_a1(int a) const
{
	return bimap_A1.right.at(bimap_A.left.at(a));
}

int preprocess_info::a_to_a2(int a) const
{
	return bimap_A2.right.at(bimap_A.left.at(a));
}

int preprocess_info::a1_to_a(int a1) const
{
	return bimap_A.right.at(bimap_A1.left.at(a1));
}

int preprocess_info::a2_to_a(int a2) const
{
	return bimap_A.right.at(bimap_A2.left.at(a2));
}

int preprocess_info::src_dst_to_a(int src, int dst) const
{
	return bimap_A.right.at(make_pair(src, dst));
}

int preprocess_info::src_dst_to_a1(int src, int dst) const
{
	return bimap_A1.right.at(make_pair(src, dst));
}

int preprocess_info::src_dst_to_a2(int src, int dst) const
{
	return bimap_A2.right.at(make_pair(src, dst));
}

void preprocess_info::prune(int orig, int dest)
{
	map<int, set<int>> forward;
	map<int, set<int>> backward;

	for (int a : A) {
		auto pair = bimap_A.left.at(a);
		int src = pair.first, dst = pair.second;

		forward[src].insert(dst);
		backward[dst].insert(src);
	}

	// Remove all nodes that don't have any outward arc or inward arc
	while (true) {
		set<int> to_be_removed;
		for (int i : V)
			if (i != orig && i != dest && (forward[i].empty() || backward[i].empty()))
				to_be_removed.insert(i);

		if (to_be_removed.empty())
			break;

		for (int i : to_be_removed) {
			V.erase(i);

			for (int j : forward[i]) {
				int a = src_dst_to_a(i, j);
				A.erase(a);
				if (is_tolled.at(a)) A1.erase(a_to_a1(a));
				else A2.erase(a_to_a2(a));
				backward[j].erase(i);
			}

			for (int j : backward[i]) {
				int a = src_dst_to_a(j, i);
				A.erase(a);
				if (is_tolled.at(a)) A1.erase(a_to_a1(a));
				else A2.erase(a_to_a2(a));
				forward[j].erase(i);
			}

			forward[i].clear();
			backward[i].clear();
		}
	}

}

void preprocess_info::reduce(int orig, int dest)
{
	clean();

	map<int, set<int>> forward;
	map<int, set<int>> backward;

	for (int a : A) {
		auto pair = bimap_A.left.at(a);
		int src = pair.first, dst = pair.second;

		forward[src].insert(dst);
		backward[dst].insert(src);
	}

	// Remove chain of toll-free arcs
	map<int, pair<int, int>> chain_nodes;
	for (int i : V) {
		if (i != orig && i != dest && forward[i].size() == 1 && backward[i].size() == 1) {
			// Test if both arcs are toll-free
			int j = *backward[i].begin();
			int k = *forward[i].begin();

			if (is_tolled.at(src_dst_to_a(j, i))) continue;
			if (is_tolled.at(src_dst_to_a(i, k))) continue;

			// Test if they are not connected
			if (forward[j].count(k)) continue;

			chain_nodes.emplace(i, make_pair(j, k));
		}
	}

	while (!chain_nodes.empty()) {
		auto entry = *chain_nodes.begin();
		chain_nodes.erase(entry.first);

		// Start and end node of the chain
		int root = entry.first;
		int start = entry.second.first;
		int end = entry.second.second;
		cost_type cost = cost_A2.at(src_dst_to_a2(start, root)) + cost_A2.at(src_dst_to_a2(root, end));

		int root_a = src_dst_to_a(start, root);
		int root_a2 = src_dst_to_a2(start, root);

		V.erase(root);
		A.erase(src_dst_to_a(root, end));
		A2.erase(src_dst_to_a2(root, end));

		// Trace the end
		while (chain_nodes.count(end)) {
			int new_end = chain_nodes[end].second;

			// If there is already an arc between start and new_end, stop here to prevent overriding existing arc
			if (backward[new_end].count(start)) {
				chain_nodes.erase(end);
				break;
			}
			else {
				V.erase(end);
				cost += cost_A2.at(src_dst_to_a2(end, new_end));
				A.erase(src_dst_to_a(end, new_end));
				A2.erase(src_dst_to_a2(end, new_end));

				chain_nodes.erase(end);
				end = new_end;
			}
		}

		// Trace back the start
		while (chain_nodes.count(start)) {
			int new_start = chain_nodes[start].first;

			// If there is already an arc between new_start and end, stop here to prevent overriding existing arc
			if (forward[new_start].count(end)) {
				chain_nodes.erase(start);
				break;
			}
			else {
				V.erase(start);
				cost += cost_A2.at(src_dst_to_a2(new_start, start));
				A.erase(src_dst_to_a(new_start, start));
				A2.erase(src_dst_to_a2(new_start, start));

				chain_nodes.erase(start);
				start = new_start;
			}
		}

		// Modify the root arc
		bimap_A.left.replace_data(bimap_A.left.find(root_a), make_pair(start, end));
		bimap_A2.left.replace_data(bimap_A2.left.find(root_a2), make_pair(start, end));

		auto trace = bimap_A2.left.at(root_a2);

		cost_A[root_a] = cost_A2[root_a2] = cost;
	}
}

void preprocess_info::clean()
{
	// Clear unlisted bimap and map entry
#define CLEAR_UNLISTED(map, set, var) \
	for (auto it = map.begin(); it != map.end();) { \
		if (!set.count(it->var)) \
			it = map.erase(it); \
		else \
			it++; \
	}

	CLEAR_UNLISTED(bimap_A, A, left);
	CLEAR_UNLISTED(bimap_A1, A1, left);
	CLEAR_UNLISTED(bimap_A2, A2, left);

	CLEAR_UNLISTED(cost_A, A, first);
	CLEAR_UNLISTED(cost_A1, A1, first);
	CLEAR_UNLISTED(cost_A2, A2, first);

	CLEAR_UNLISTED(is_tolled, A, first);
}

light_graph preprocess_info::build_graph()
{
	light_graph lgraph(*(V.rbegin()) + 1);

	problem_base::edge_iterator ei, ei_end;
	for (int a : A) {
		auto arc = bimap_A.left.at(a);
		cost_type cost = cost_A.at(a);
		bool tolled = is_tolled.at(a);
		int index = tolled ? a_to_a1(a) : -1;

		lgraph.Eall.emplace_back(light_edge{
			.index = index,
			.src = arc.first,
			.dst = arc.second,
			.cost = cost,
			.is_tolled = tolled,
			.toll = 0,
			.enabled = true,
			.temp_enabled = true
								 });
	}

	LOOP(a, lgraph.Eall.size()) {
		auto& edge = lgraph.Eall[a];
		lgraph.E[edge.src].emplace(edge.dst, a);
		lgraph.Er[edge.dst].emplace(edge.src, a);
	}

	return lgraph;
}

preprocess_info preprocessor::preprocess(const problem& prob, int k) {
	return preprocess_impl(prob.graph, prob.commodities[k], k);
}

// Default proprocessor: extract problem to preprocess_info
preprocess_info preprocessor::preprocess_impl(const light_graph& graph, const commodity& comm, int k)
{
	using arc_bimap_rel = preprocess_info::arc_bimap::value_type;

	preprocess_info info;

	LOOP(i, graph.V) info.V.insert(i);

	int a2 = 0;
	LOOP(a, graph.Eall.size()) {
		info.A.insert(a);

		const light_edge& e = graph.Eall.at(a);
		cost_type cost = e.cost;
		bool is_tolled = e.is_tolled;

		info.bimap_A.insert(arc_bimap_rel(a, make_pair(e.src, e.dst)));
		info.cost_A.emplace(a, cost);
		info.is_tolled.emplace(a, is_tolled);

	 	if (is_tolled) {
			info.bimap_A1.insert(arc_bimap_rel(e.index, make_pair(e.src, e.dst)));
			info.cost_A1.emplace(e.index, cost);
			info.A1.insert(e.index);
		}
		else {
			info.bimap_A2.insert(arc_bimap_rel(a2, make_pair(e.src, e.dst)));
			info.cost_A2.emplace(a2, cost);
			info.A2.insert(a2);
			a2++;
		}
	}

	return info;
}
