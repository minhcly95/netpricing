#pragma once

#include "../base/formulation.h"

struct light_graph;

struct vfpath_formulation : public formulation {
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
	VarMatrix tz;

	// Constraints
	IloRange sum_z;
	RangeArray sum_tz;
	RangeMatrix upperbound;
	RangeArray valuefunc;

	// Index of the callback optimal path
	int cb_opt_p;
	cost_type cb_opt_toll;

	vfpath_formulation(const std::vector<path>& paths, bool full_mode);
	virtual ~vfpath_formulation();

	void prepare();
	IloRange get_cut(int target_p);

	virtual void formulate_impl() override;
	virtual std::vector<IloNumVar> get_all_variables() override;
	virtual IloExpr get_obj_expr() override;

	virtual std::vector<std::pair<IloNumVar, IloNum>> path_to_solution(const NumArray& tvals,
																  const std::vector<int>& path) override;

	virtual bool has_callback() override { return !full_mode; }
	virtual void invoke_callback(const IloCplex::Callback::Context& context, const NumArray& tvals) override;

	virtual bool has_callback_optimal_path() override { return !full_mode; }
	virtual std::vector<int> get_callback_optimal_path() override;
	virtual double get_callback_obj() override;

	virtual bool has_heuristic_cut() override { return !full_mode; }
	virtual void post_heuristic_cut(const IloCplex::Callback::Context& context, const NumArray& tvals,
									const std::vector<int>& path) override;
};