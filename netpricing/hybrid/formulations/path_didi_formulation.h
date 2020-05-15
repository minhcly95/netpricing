#pragma once

#include "path_based_formulation.h"

struct light_graph;

struct path_didi_formulation : public path_based_formulation {
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
	RangeArray tk_upper;

	path_didi_formulation(const std::vector<path>& paths, bool full_mode);
	virtual ~path_didi_formulation();

	virtual void formulate_impl() override;
	virtual std::vector<IloNumVar> get_all_variables() override;
	virtual IloExpr get_obj_expr() override;

	virtual std::vector<std::pair<IloNumVar, IloNum>> path_to_solution(const NumArray& tvals,
																	   const std::vector<int>& path) override;
};