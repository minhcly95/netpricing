#pragma once

#include "processed_formulation.h"

struct value_func_formulation : public processed_formulation {
	constexpr static double TOLERANCE = 1e-6;

	VarArray x;
	VarArray y;
	VarArray tx;

	RangeArray flow_constr;
	RangeArray bilinear1;
	RangeArray bilinear2;
	RangeArray bilinear3;

	std::vector<int> cb_path;

	// Unprocessed version
	value_func_formulation();
	// Processed version
	value_func_formulation(preprocessor* _preproc);

	virtual ~value_func_formulation();

	IloRange get_cut(const std::vector<int>& path);

	virtual void formulate_impl() override;
	virtual std::vector<IloNumVar> get_all_variables() override;
	virtual IloExpr get_obj_expr() override;

	virtual std::vector<std::pair<IloNumVar, IloNum>> path_to_solution(const NumArray& tvals,
																  const std::vector<int>& path) override;

	virtual bool has_callback() override { return true; }
	virtual void invoke_callback(const IloCplex::Callback::Context& context, const NumArray& tvals) override;

	virtual bool has_callback_optimal_path() override { return true; }
	virtual std::vector<int> get_callback_optimal_path() override;
	virtual double get_callback_obj() override;

	virtual bool has_heuristic_cut() override { return true; }
	virtual void post_heuristic_cut(const IloCplex::Callback::Context& context, const NumArray& tvals,
									const std::vector<int>& path) override;
};