#pragma once

#include "../model.h"

struct csenum;

enum csenum_branch_dir : bool {
	PRIMAL,
	DUAL
};

struct csenum_coor {
	int k;
	int a;
};

struct csenum_branch {
	int k;
	int a;
	csenum_branch_dir dir;
};

struct csenum_node {
	static constexpr double TOLERANCE = 1e-4;

	csenum* cse;
	int index;
	int parent;

	double bound;
	std::vector<double> primal_objs;
	double dual_obj;

	std::vector<csenum_branch> lineage;

	std::vector<std::vector<int>> paths;
	cplex_def::NumMatrix lambda;
	cplex_def::NumArray t;

	// Updated later
	std::vector<std::vector<int>> violated;

	void update_bound();
	int get_depth() const;			// The length of lineage

	bool is_feasible() const;	
	void update_violated();			// Get all coors (k,a) where comp-slack constraint is violated
	void update_violated(int k);
	int get_num_violated() const;

	bool operator<(const csenum_node& other) const;
};
