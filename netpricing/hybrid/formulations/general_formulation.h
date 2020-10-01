#pragma once

#include "../base/formulation.h"
#include "../preprocessors/path_preprocessor.h"

struct light_graph;

struct general_formulation : public formulation {
	constexpr static double TOLERANCE = 1e-4;
	constexpr static double TOLL_PREFERENCE = 0.9999;
	//constexpr static double TOLLFREE_PENALTY = 0.0001;

	enum space : bool {
		ARC = false,
		PATH = true
	};
	enum opt_condition : bool {
		STRONG_DUAL = false,
		COMP_SLACK = true
	};
	enum linearization_method : bool {
		DIRECT = false,
		SUBSTITUTION = true
	};

	// SWITCHES
	space primal_space;
	space dual_space;
	opt_condition opt_cond;
	linearization_method linearization;

	// VARIABLES
	VarArray x;			// Primal Arc
	VarArray y;
	VarArray z;			// Primal Path

	VarArray lambda;	// Dual Arc
	IloNumVar lk;		// Dual Path

	VarArray tx;		// Direct linearization
	IloNumVar tk;		// Strong duality substitution linearization

	// CONSTRAINTS
	RangeArray flow_constr;		// Primal Arc
	IloRange sum_z;				// Primal Path

	RangeArray dual_feas;		// Dual Arc
	RangeArray valuefunc;		// Dual Path

	IloRange strong_dual;		// Strong duality (either opt condition or substitution)
	RangeArray comp_slack;		// Complementary slackness

	RangeArray bilinear1;		// Substitution linearization
	RangeArray bilinear2;
	RangeArray bilinear3;

	// Preprocessor
	path_preprocessor preproc;
	preprocess_info info;
	light_graph original;
	light_graph* lgraph;

	// Path attributes
	using path = std::vector<int>;
	using arc_set = std::set<int>;

	int P;
	std::vector<path> paths;
	std::vector<cost_type> null_costs;
	std::vector<arc_set> toll_sets;
	std::vector<arc_set> arc_sets;

	general_formulation(const std::vector<path>& paths, const light_graph& original,
						space primal_space, space dual_space,
						opt_condition opt_cond, linearization_method linearization);
	general_formulation(const std::vector<path>& paths, const light_graph& original, const std::string& code);
	virtual ~general_formulation();

	void prepare();
	bool check_model(space primal, space dual, opt_condition condition);

	virtual void formulate_impl() override;
	virtual std::vector<IloNumVar> get_all_variables() override;
	virtual IloExpr get_obj_expr() override;

	virtual std::vector<int> get_optimal_path() override;

	virtual std::vector<std::pair<IloNumVar, IloNum>> path_to_solution(const NumArray& tvals,
																	   const std::vector<int>& path) override;

	virtual bool has_callback() override;
	virtual void invoke_callback(const IloCplex::Callback::Context& context, const NumArray& tvals) override;

	// Helper functions
	std::vector<int> extract_path_from_src_dst_map(const std::multimap<int, int>& src_dst_map);
};