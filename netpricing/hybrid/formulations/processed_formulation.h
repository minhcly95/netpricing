#pragma once

#include "../base/formulation.h"
#include <set>

struct light_graph;

struct processed_formulation : public formulation {
	using path = std::vector<int>;

	int P;
	std::vector<path> paths;

	std::set<int> removed_V;
	std::set<int> removed_A;
	std::set<int> removed_A1;
	std::set<int> removed_A2;

	// Unprocessed version
	processed_formulation();

	// Processed version
	processed_formulation(const std::vector<path>& paths);

	virtual ~processed_formulation();

	void process_graph();

	bool is_valid_V(int i) { return removed_V.find(i) == removed_V.end(); }
	bool is_valid_A(int a) { return removed_A.find(a) == removed_A.end(); }
	bool is_valid_A1(int a) { return removed_A1.find(a) == removed_A1.end(); }
	bool is_valid_A2(int a) { return removed_A2.find(a) == removed_A2.end(); }
};

#define LOOP_VALID(i, size) LOOP(i, size) if (is_valid_ ## size(i))
