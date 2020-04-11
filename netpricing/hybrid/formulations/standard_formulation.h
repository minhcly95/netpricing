#pragma once

#include "../base/formulation.h"

struct standard_formulation : public formulation {
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

	virtual void formulate_impl() override;
};