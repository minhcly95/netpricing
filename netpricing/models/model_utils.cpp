#include "model_utils.h"

#include <map>

#include "../macros.h"

using namespace std;

template <class ArrayType>
IloArray<ArrayType> make_matrix(IloEnv env, int d1, int d2) {
	IloArray<ArrayType> matrix(env, d1);
	LOOP(i, d1) matrix[i] = ArrayType(env, d2);
	return matrix;
}

model_cplex::NumMatrix make_num_matrix(IloEnv env, int d1, int d2) {
	return make_matrix<model_cplex::NumArray>(env, d1, d2);
}

model_cplex::NumVarMatrix make_numvar_matrix(IloEnv env, int d1, int d2) {
	return make_matrix<model_cplex::NumVarArray>(env, d1, d2);
}

model_cplex::RangeMatrix make_range_matrix(IloEnv env, int d1, int d2) {
	return make_matrix<model_cplex::RangeArray>(env, d1, d2);
}

model_cplex::NumArray get_values(IloCplex& cplex, const model_cplex::NumVarArray& vars)
{
	model_cplex::NumArray vals(cplex.getEnv(), vars.getSize());
	cplex.getValues(vals, vars);
	return vals;
}

model_cplex::NumMatrix get_values(IloCplex& cplex, const model_cplex::NumVarMatrix& vars)
{
	int K = vars.getSize();
	model_cplex::NumMatrix vals(cplex.getEnv(), K);
	LOOP(k, K) {
		vals[k] = get_values(cplex, vars[k]);
	}
	return vals;
}

void clean_up(model_cplex::NumVarMatrix& vars)
{
	int K = vars.getSize();
	LOOP(k, K) vars[k].end();
	vars.end();
}

void clean_up(model_cplex::NumMatrix& vals)
{
	int K = vals.getSize();
	LOOP(k, K) vals[k].end();
	vals.end();
}

map<int, int> src_dst_map_from_z(const model_single& m, const model_cplex::NumArray& zvals) {
	map<int, int> src_dst_map;
	LOOP(a, m.A) {
		SRC_DST_FROM_A(m.prob, a);
		if (zvals[a] > 0.5)
			src_dst_map[src] = dst;
	}
	return src_dst_map;
}

map<int, int> src_dst_map_from_xy(const model_single& m, const model_cplex::NumArray& xvals, const model_cplex::NumArray& yvals) {
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

solution fetch_solution_from_z_t(const model_single& m, model_cplex::NumMatrix& zvals, model_cplex::NumArray& tvals)
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

solution fetch_solution_from_xy_t(const model_single& m, model_cplex::NumMatrix& xvals, model_cplex::NumMatrix& yvals, model_cplex::NumArray& tvals)
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
