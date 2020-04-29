#pragma once

#include "../base/formulation.h"

struct light_graph;

struct path_didi_formulation : public formulation {
	constexpr static double TOLERANCE = 1e-6;
	constexpr static double TOLL_PREFERENCE = 0.9999;

	using path = std::vector<int>;
	using toll_set = std::set<int>;		// Set of indices of toll arcs belonging to the path

	// Path attributes
	int P;
	std::vector<path> paths;
	std::vector<cost_type> null_costs;
	std::vector<toll_set> toll_sets;

	bool full_mode;

	// Variables
	VarArray z;
	IloNumVar lk;
	IloNumVar tk;

	// Constraints
	IloRange sum_z;
	RangeArray lk_upper;
	RangeArray lk_lower;
	IloRange tk_constr;

	path_didi_formulation(const std::vector<path>& paths, bool full_mode);
	virtual ~path_didi_formulation();

	void prepare();

	virtual void formulate_impl() override;
	virtual std::vector<IloNumVar> get_all_variables() override;
	virtual IloExpr get_obj_expr() override;

	virtual std::vector<std::pair<IloNumVar, IloNum>> path_to_solution(const NumArray& tvals,
																	   const std::vector<int>& path) override;
};