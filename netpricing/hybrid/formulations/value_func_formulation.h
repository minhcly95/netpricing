#pragma once

#include "../base/formulation.h"

struct light_graph;

struct value_func_formulation : public formulation {
	constexpr static double TOLERANCE = 1e-6;
	constexpr static double TOLL_PREFERANCE = 0.9999;

	VarArray x;
	VarArray y;
	VarArray tx;

	RangeArray flow_constr;
	RangeArray bilinear1;
	RangeArray bilinear2;
	RangeArray bilinear3;

	light_graph* lgraph;
	std::vector<int> cb_path;

	virtual ~value_func_formulation();

	virtual void formulate_impl() override;

	virtual bool has_callback() override { return true; }
	virtual void invoke_callback(const IloCplex::Callback::Context& context, const NumArray& tvals) override;

	virtual bool has_callback_solution() override { return true; }
	virtual std::vector<std::pair<IloNumVar, IloNum>> get_callback_solution(const NumArray& tvals) override;
	virtual double get_callback_obj() override;
};