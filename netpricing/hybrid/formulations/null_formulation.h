#pragma once

#include "../base/formulation.h"

struct null_formulation : public formulation {
	virtual void formulate_impl() override {}
	virtual std::vector<IloNumVar> get_all_variables() override { return {}; }
	virtual IloExpr get_obj_expr() override { return IloExpr(env); }

	virtual std::vector<std::pair<IloNumVar, IloNum>> path_to_solution(const NumArray& tvals,
																	   const std::vector<int>& path) override {
		return {};
	}
};