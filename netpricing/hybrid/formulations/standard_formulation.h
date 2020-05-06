#pragma once

#include "processed_formulation.h"

struct standard_formulation : public processed_formulation {
	VarArray x;
	VarArray y;
	VarArray lambda;
	VarArray tx;

	RangeArray flow_constr;
	RangeArray dual_feas;
	IloRange equal_obj;
	RangeArray bilinear1;
	RangeArray bilinear2;
	RangeArray bilinear3;

	// Unprocessed version
	standard_formulation();
	// Processed version
	standard_formulation(preprocessor* _preproc);

	virtual ~standard_formulation();

	virtual void formulate_impl() override;
	virtual std::vector<IloNumVar> get_all_variables() override;
	virtual IloExpr get_obj_expr() override;

	virtual std::vector<std::pair<IloNumVar, IloNum>> path_to_solution(const NumArray& tvals,
																  const std::vector<int>& path) override;
};