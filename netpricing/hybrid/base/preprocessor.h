#pragma once

#include <vector>
#include <set>
#include <map>
#include <boost/bimap.hpp>
#include "../../typedef.h"
#include "../../graph/light_graph.h"

struct problem;

struct preprocess_info {
	using iset = std::set<int>;
	using arc = std::pair<int, int>;
	using arc_bimap = boost::bimap<int, arc>;
	using arc_bimap_rel = arc_bimap::value_type;

	template <typename value_type>
	using imap = std::map<int, value_type>;

	iset V, A, A1, A2;

	arc_bimap bimap_A, bimap_A1, bimap_A2;

	imap<cost_type> cost_A, cost_A1, cost_A2;
	imap<bool> is_tolled;

	// Conversion
	int a_to_a1(int a) const;
	int a_to_a2(int a) const;
	int a1_to_a(int a1) const;
	int a2_to_a(int a2) const;

	int src_dst_to_a(int src, int dst) const;
	int src_dst_to_a1(int src, int dst) const;
	int src_dst_to_a2(int src, int dst) const;

	void reduce(int orig, int dest);

	light_graph build_graph();
};

struct preprocessor {
	virtual preprocess_info preprocess(const problem& prob, int k);
};

#define LOOP_INFO(index, set) for (auto index : info.set)
