#pragma once

#include "path_based_formulation.h"
#include "../preprocessors/path_preprocessor.h"

struct light_graph;

struct arc_path_standard_formulation : public path_based_formulation {
	constexpr static double TOLERANCE = 1e-5;

	// Variables
	VarArray z;
	VarArray lambda;
	VarArray tx;

	// Constraint
	IloRange sum_z;
	RangeArray dual_feas;
	IloRange equal_obj;
	RangeArray bilinear1;
	RangeArray bilinear2;
	RangeArray bilinear3;

	path_preprocessor preproc;
	preprocess_info info;
	light_graph* lgraph;

	arc_path_standard_formulation(const std::vector<path>& paths);
	virtual ~arc_path_standard_formulation();

	virtual void formulate_impl() override;
	virtual std::vector<IloNumVar> get_all_variables() override;
	virtual IloExpr get_obj_expr() override;

	virtual std::vector<std::pair<IloNumVar, IloNum>> path_to_solution(const NumArray& tvals,
																	   const std::vector<int>& path) override;
};