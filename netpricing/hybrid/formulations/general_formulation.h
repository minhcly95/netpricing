#pragma once

#include "../base/formulation.h"
#include "../preprocessors/path_preprocessor.h"

struct light_graph;

struct general_formulation : public formulation {
	constexpr static double TOLERANCE = 1e-4;
	constexpr static double TOLL_PREFERENCE = 0.9999;

	enum space : bool {
		ARC = false,
		PATH = true
	};
	enum opt_condition : bool {
		STRONG_DUAL = false,
		COMP_SLACK = true
	};

	// SWITCHES
	space primal_space;
	space dual_space;
	opt_condition opt_cond;

	// VARIABLES
	VarArray x;			// Primal Arc
	VarArray y;
	VarArray z;			// Primal Path

	VarArray lambda;	// Dual Arc
	IloNumVar lk;		// Dual Path

	VarArray tx;		// Strong duality
	IloNumVar tk;		// Complementary slackness

	// CONSTRAINTS
	RangeArray flow_constr;		// Primal Arc
	IloRange sum_z;				// Primal Path

	RangeArray dual_feas;		// Dual Arc
	RangeArray valuefunc;		// Dual Path

	IloRange strong_dual;		// Strong duality (CS uses it to extract leader revenue)
	RangeArray comp_slack;		// Complementary slackness

	RangeArray bilinear1;
	RangeArray bilinear2;
	RangeArray bilinear3;

	// Preprocessor
	path_preprocessor preproc;
	preprocess_info info;
	light_graph* lgraph;

	// Path attributes
	using path = std::vector<int>;
	using arc_set = std::set<int>;

	int P;
	std::vector<path> paths;
	std::vector<cost_type> null_costs;
	std::vector<arc_set> toll_sets;
	std::vector<arc_set> arc_sets;

	general_formulation(const std::vector<path>& paths, space primal_space, space dual_space, opt_condition opt_cond);
	general_formulation(const std::vector<path>& paths, const std::string& code);
	virtual ~general_formulation();

	void prepare();
	bool check_model(space primal, space dual, opt_condition condition);

	virtual void formulate_impl() override;
	virtual std::vector<IloNumVar> get_all_variables() override;
	virtual IloExpr get_obj_expr() override;

	virtual std::vector<std::pair<IloNumVar, IloNum>> path_to_solution(const NumArray& tvals,
																	   const std::vector<int>& path) override;
};