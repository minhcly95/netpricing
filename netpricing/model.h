#pragma once

#include "problem.h"
#include "problem_multi.h"
#include "solution.h"

#include <utility>
#include <chrono>
#include <ilcplex/ilocplex.h>
#include <vector>
#include <sstream>

struct model_config {
	int num_thread;
	int var_select;
	int time_limit;
	int heur_freq;
	int pre_cut;
	bool full_mode;
	int max_paths;
};

struct model_base {
	std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
	double time;

	virtual bool solve_impl() = 0;
	bool solve() {
		start_time = std::chrono::high_resolution_clock::now();
		bool res = solve_impl();
		auto end = std::chrono::high_resolution_clock::now();
		time = std::chrono::duration<double>(end - start_time).count();
		return res;
	}

	virtual void config(const model_config& config) = 0;

	virtual void end() { }

	double get_time() {
		return time;
	}

	virtual solution get_solution() = 0;

	virtual double get_best_obj() { return -1; }
	virtual double get_best_bound() { return -1; }
	virtual double get_gap() { return -1; }
	virtual int get_step_count() { return -1; }

	virtual std::string get_report() {
		using namespace std;
		ostringstream ss;
		ss <<
			"TIME: " << get_time() << " s" << endl <<
			"OBJ: " << get_best_obj() << endl <<
			"BOUND: " << get_best_bound() << endl <<
			"GAP: " << get_gap() * 100 << " %" << endl <<
			"STEP: " << get_step_count() << endl;

		return ss.str();
	}
};

struct model_single {
	using problem_type = problem;

	problem prob;
	int K, V, A, A1, A2;

	model_single(const problem& _prob) : prob(_prob) {
		K = prob.commodities.size();
		V = boost::num_vertices(prob.graph);
		A = boost::num_edges(prob.graph);
		A1 = prob.tolled_index_map.size();
		A2 = prob.tollfree_index_map.size();
	}
};

struct model_multi {
	using problem_type = problem_multi;

	problem_multi prob;
	int K, V, A, A1;
	std::vector<int> A2;

	model_multi(const problem_multi& _prob) : prob(_prob) {
		K = prob.commodities.size();
		V = boost::num_vertices(prob.graphs[0]);
		A = prob.max_edge_index;
		A1 = prob.tolled_index_common_map.size();

		for (auto& map : prob.tollfree_index_maps) {
			A2.push_back(map.size());
		}
	}
};

struct cplex_def {
	using NumVarArray = IloNumVarArray;
	using NumVarMatrix = IloArray<NumVarArray>;
	using VarArray = NumVarArray;
	using VarMatrix = NumVarMatrix;

	using NumArray = IloNumArray;
	using NumMatrix = IloArray<NumArray>;

	using RangeArray = IloRangeArray;
	using RangeMatrix = IloArray<RangeArray>;
	using ConstraintArray = IloConstraintArray;
};
