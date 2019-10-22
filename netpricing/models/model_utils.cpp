#include "model_utils.h"

#include <map>

#include "../macros.h"

using namespace std;

model::NumArray get_values(IloCplex& cplex, const model::NumVarArray& vars)
{
	model::NumArray vals(cplex.getEnv(), vars.getSize());
	cplex.getValues(vals, vars);
	return vals;
}

model::NumMatrix get_values(IloCplex& cplex, const model::NumVarMatrix& vars)
{
	int K = vars.getSize();
	model::NumMatrix vals(cplex.getEnv(), K);
	LOOP(k, K) {
		vals[k] = get_values(cplex, vars[k]);
	}
	return vals;
}

void clean_up(model::NumVarMatrix& vars)
{
	int K = vars.getSize();
	LOOP(k, K) vars[k].end();
	vars.end();
}

void clean_up(model::NumMatrix& vals)
{
	int K = vals.getSize();
	LOOP(k, K) vals[k].end();
	vals.end();
}

map<int, int> src_dst_map_from_z(const model& m, const model::NumArray& zvals) {
	map<int, int> src_dst_map;
	LOOP(a, m.A) {
		SRC_DST_FROM_A(m.prob, a);
		if (zvals[a] > 0.5)
			src_dst_map[src] = dst;
	}
	return src_dst_map;
}

map<int, int> src_dst_map_from_xy(const model& m, const model::NumArray& xvals, const model::NumArray& yvals) {
	map<int, int> src_dst_map;
	LOOP(a, m.A1) {
		SRC_DST_FROM_A1(m.prob, a);
		if (xvals[a] > 0.5)
			src_dst_map[src] = dst;
	}
	LOOP(a, m.A2) {
		SRC_DST_FROM_A2(m.prob, a);
		if (yvals[a] > 0.5)
			src_dst_map[src] = dst;
	}
	return src_dst_map;
}

solution::path path_from_src_dst_map(const commodity& c, const map<int, int>& src_dst_map) {
	solution::path path;

	int current = c.origin;
	path.push_back(current);
	while (current != c.destination) {
		current = src_dst_map.at(current);
		path.push_back(current);
	}

	return path;
}

solution fetch_solution_from_z_t(const model& m, model::NumMatrix& zvals, model::NumArray& tvals)
{
	solution sol;

	// Paths
	LOOP(k, m.K) {
		sol.paths.push_back(path_from_src_dst_map(m.prob.commodities[k],
												  src_dst_map_from_z(m, zvals[k])));
	}

	// Tolls
	LOOP(a, m.A1) {
		sol.tolls.push_back(tvals[a]);
	}

	return sol;
}

solution fetch_solution_from_xy_t(const model& m, model::NumMatrix& xvals, model::NumMatrix& yvals, model::NumArray& tvals)
{
	solution sol;

	// Paths
	LOOP(k, m.K) {
		sol.paths.push_back(path_from_src_dst_map(m.prob.commodities[k],
												  src_dst_map_from_xy(m, xvals[k], yvals[k])));
	}

	// Tolls
	LOOP(a, m.A1) {
		sol.tolls.push_back(tvals[a]);
	}

	return sol;
}
