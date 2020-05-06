#pragma once

#include <vector>
#include <map>
#include <boost/bimap.hpp>
#include "../../typedef.h"

struct problem;

struct preprocess_info {
	using ivec = std::vector<int>;
	using arc = std::pair<int, int>;
	using arc_bimap = boost::bimap<int, arc>;

	template <typename value_type>
	using imap = std::map<int, value_type>;

	ivec V, A, A1, A2;

	arc_bimap bimap_A, bimap_A1, bimap_A2;

	imap<cost_type> cost_A, cost_A1, cost_A2;
	imap<bool> is_tolled;

	int a_to_a1(int a);
	int a_to_a2(int a);
	int a1_to_a(int a1);
	int a2_to_a(int a2);

	int src_dst_to_a(int src, int dst);
	int src_dst_to_a1(int src, int dst);
	int src_dst_to_a2(int src, int dst);
};

struct preprocessor {
	virtual preprocess_info preprocess(const problem& prob);
};

#define LOOP_INFO(index, set) for (auto index : info.set)
