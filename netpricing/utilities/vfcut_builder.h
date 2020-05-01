#pragma once

#include "../model.h"
#include "follower_light_solver.h"

#include <vector>

struct vfcut_builder : public cplex_def
{
	constexpr static double TOLERANCE = 1e-6;

	IloEnv env;
	follower_light_solver solver;
	problem& prob;

	VarMatrix x;
	VarMatrix y;
	VarArray t;
	VarMatrix tx;

	double time;
	int count;

	vfcut_builder(IloEnv& env, const problem& prob,
				  const VarMatrix& x,
				  const VarMatrix& y,
				  const VarArray& t,
				  const VarMatrix& tx);

	void build(const NumArray& tvals, RangeArray& cuts, NumMatrix& xvals, NumMatrix& yvals, IloNum& obj);
	IloRange build_cut(const std::vector<int>& path, int k);

	double get_sub_time() const;
	void reset();
};
