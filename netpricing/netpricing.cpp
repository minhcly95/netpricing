// netpricing.cpp : Defines the entry point for the application.
//

#include "netpricing.h"
#include "netpricing_options.h"

#include <chrono>
#include <string>
#include <fstream>
#include <sstream>
#include <thread>
#include <regex>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/program_options.hpp>

#include "nlohmann/json.hpp"

using namespace std;
using namespace boost;
using json = nlohmann::json;
namespace po = boost::program_options;

struct config {
	json& sols_obj;
	model_config mconfig;
};

template <class P, class T>
void print_src_dst(P& prob, T& model) {
	LOOP(a, model.A1) {
		SRC_DST_FROM_A1(prob, a);
		cout << src << "->" << dst << " ";
	}
	cout << endl;
	LOOP(a, model.A2) {
		SRC_DST_FROM_A2(prob, a);
		cout << src << "->" << dst << " ";
	}
	cout << endl;
}

template <class model_type, class ... args_type>
string run_model(IloEnv& env, typename model_type::problem_type& prob, string model_name, config& conf, args_type... args) {
	//try {
		cout << "--------------------------------------" << endl;
		cout << model_name << endl;

		model_type model(env, prob, args...);
		model.config(conf.mconfig);

		if (!model.solve()) {
			env.error() << "Failed to optimize LP." << endl;
			throw(-1);
		}

		/*env.out() << "Solution status = " << cplex.getStatus() << endl;
		env.out() << "Solution value = " << cplex.getObjValue() << endl;*/
		string report = model.get_report();

		if (!conf.mconfig.relax_only) {
			json sol_obj = model.get_solution().get_json(prob);
			sol_obj["name"] = model_name;
			conf.sols_obj.push_back(std::move(sol_obj));
		}

		model.end();

		regex report_regex("^|(\\n)(?!$)");
		report = std::regex_replace(report, report_regex, "$1  ");
		return report;
	//}
	//catch (const IloException & e) {
	//	cerr << "Exception caught: " << e << endl;
	//}
}

void run_routine(int index, const vector<string>& args);

int main(int argc, char* argv[])
{
	const int DEFAULT_NUM_THREADS = thread::hardware_concurrency();

	// Parameter processing
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "display help message")
		("routine,r", po::value<int>(), "run special routines")
		("args", po::value<vector<string>>(), "other arguments")

		("standard,s", "run standard model")
		("vfcut", "run standard model with value function cuts")
		("lvfcut", "run standard model with value function cuts (light version)")
		("goal", "run standard model with CPLEX goals")
		("cscut", "run standard model with complementary slackness cuts")
		("benders", po::value<int>(), "run Benders model (arg is 0, 1, or 2)")
		("valuefunc", "run value function model")
		("xtbenders", "run xT Benders model")
		("xybenders", "run xy Benders model")
		("xytbenders", "run xyT Benders model")
		("slackbr", "run slack-branch model")
		("csenum", "run complementary slackness enumeration")
		("csenum-excl", "run complementary slackness enumeration (exclusive branch version)")
		("compslack", "run complementary slackness model")

		("path-std", "run path model (fallback to standard model)")
		("path-vf", "run path model (fallback to value function model)")
		("vfpath-std", "run vfpath model (fallback to standard model)")
		("vfpath-vf", "run vfpath model (fallback to value function model)")
		("didi-std", "run Didi's path model (fallback to standard model)")
		("didi-vf", "run Didi's path model (fallback to value function model)")
		("fstandard", "run filtered standard model")
		("fvaluefunc", "run filtered value function model")
		("pstandard", "run processed standard model")
		("pvaluefunc", "run processed value function model")
		("spgm-std", "run SPGM standard model")
		("spgm-vf", "run SPGM value function model")
		("p-spgm-std", "run processed - SPGM standard model")
		("p-spgm-vf", "run processed - SPGM value function model")
		("apstd-std", "run arc-path standard model (fallback to standard model)")
		("apstd-vf", "run arc-path standard model (fallback to value function model)")
		("apvf-std", "run arc-path value function model (fallback to standard model)")
		("apvf-vf", "run arc-path value function model (fallback to value function model)")

		("compose,c", po::value<string>(), "run hybrid composed model")

		("multi", "run multi-graph version")
		("hybrid,H", "run hybrid version")

		("input,i", po::value<string>(), "input problem from file")
		("output,o", po::value<string>()->default_value("report.json"), "output report file")

		("thread,t", po::value<int>()->default_value(DEFAULT_NUM_THREADS), "number of threads")
		("var-select,v", po::value<int>()->default_value(IloCplex::DefaultVarSel), "variable selection strategy")
		("time,T", po::value<int>()->default_value(0), "time limit (0 = no limit)")
		("heur-freq,h", po::value<int>()->default_value(-1), "heuristic frequency (-1 = default)")
		("pre-cut", po::value<int>()->default_value(0), "number of cuts added per commodity pre-solved")
		("full-mode,F", "add all cuts to root node")
		("max-paths,P", po::value<int>()->default_value(10), "maximum num paths for path hybrid models")
		("relax-only,R", "only solve the relaxation")
		("pre-spgm,S", "apply SPGM before path-based preprocessing")

		("nodes,n", po::value<int>()->default_value(10), "number of nodes in the random problem")
		("arcs,a", po::value<int>()->default_value(20), "number of arcs in the random problem")
		("commodities,k", po::value<int>()->default_value(5), "number of commodities in the random problem")
		("toll-proportion,p", po::value<float>()->default_value(0.2f), "toll proportion in the random problem")
		("grid", po::value<grid_params>()->multitoken(), "create a grid problem")
		("delaunay", po::value<int>(), "create a Delaunay graph problem")
		("voronoi", po::value<int>(), "create a Voronoi graph problem");

	po::positional_options_description pos_desc;
	pos_desc.add("args", -1);

	po::variables_map vm;

	try {
		po::store(po::command_line_parser(argc, argv).options(desc).positional(pos_desc).run(), vm);
		po::notify(vm);
	}
	catch (std::exception& e) {
		cerr << "Invalid option: " << e.what() << endl;
		return -1;
	}

	// Help
	if (vm.count("help")) {
		cout << desc << endl;
		return 0;
	}

	// Special routine
	if (vm.count("routine")) {
		vector<string> args;
		if (vm.count("args"))
			args = vm["args"].as<vector<string>>();
		run_routine(vm["routine"].as<int>(), args);
		return 0;
	}

	// Report json object
	json report_obj, sols_obj;

	// Random generator
	auto seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
	auto random_engine = default_random_engine(seed);

	// Create a problem
	problem* prob;
	problem_multi* prob_multi;
	if (vm.count("input")) {
		prob = new problem(problem::read_from_json(vm["input"].as<string>())[0]);
		cout << "NETWORK imported from " << vm["input"].as<string>() << endl;
	}
	else if (vm.count("grid")) {
		auto params = vm["grid"].as<grid_params>();
		int k = vm["commodities"].as<int>();
		float p = vm["toll-proportion"].as<float>();
		prob = new problem(random_grid_problem(params.width, params.height, k, p, random_engine));
		int n = num_vertices(prob->graph);
		int a = num_edges(prob->graph);

		cout << "GRID NETWORK created:" << endl;
		cout << "    width  = " << params.width << endl;
		cout << "    height = " << params.height << endl;
		cout << "    n      = " << n << endl;
		cout << "    a      = " << a << endl;
		cout << "    k      = " << k << endl;
		cout << "    p      = " << p << endl;
	}
	else if (vm.count("delaunay")) {
		int n = vm["delaunay"].as<int>();
		int k = vm["commodities"].as<int>();
		float p = vm["toll-proportion"].as<float>();
		prob = new problem(random_delaunay_problem(n, k, p, random_engine));
		int a = num_edges(prob->graph);

		cout << "DELAUNAY NETWORK created:" << endl;
		cout << "    n = " << n << endl;
		cout << "    a = " << a << endl;
		cout << "    k = " << k << endl;
		cout << "    p = " << p << endl;
	}
	else if (vm.count("voronoi")) {
		int n = vm["voronoi"].as<int>();
		int k = vm["commodities"].as<int>();
		float p = vm["toll-proportion"].as<float>();
		prob = new problem(random_voronoi_problem(n, k, p, random_engine));
		n = num_vertices(prob->graph);
		int a = num_edges(prob->graph);

		cout << "VORONOI NETWORK created:" << endl;
		cout << "    n         = " << n << endl;
		cout << "    a         = " << a << endl;
		cout << "    k         = " << k << endl;
		cout << "    p         = " << p << endl;
	}
	else {
		int n = vm["nodes"].as<int>();
		int a = vm["arcs"].as<int>();
		int k = vm["commodities"].as<int>();
		float p = vm["toll-proportion"].as<float>();
		prob = new problem(random_problem(n, a, k, p, random_engine));

		cout << "RANDOM NETWORK created:" << endl;
		cout << "    n = " << n << endl;
		cout << "    a = " << a << endl;
		cout << "    k = " << k << endl;
		cout << "    p = " << p << endl;
	}
	report_obj["problem"] = prob->get_json();

	// Problem multi
	bool is_multi = vm.count("multi") > 0;
	if (is_multi) {
		prob_multi = new problem_multi(*static_cast<problem*>(prob));
		cout << "MULTI-GRAPH version activated" << endl;
	}

	// Hybrid model
	bool is_hybrid = vm.count("hybrid") > 0;

	// Run the models
	IloEnv env;
	ostringstream report;
	const int num_thread = vm["thread"].as<int>();
	const int var_select = vm["var-select"].as<int>();
	const int time_limit = vm["time"].as<int>();
	const int heur_freq = vm["heur-freq"].as<int>();
	const int pre_cut = vm["pre-cut"].as<int>();
	const bool full_mode = vm.count("full-mode");
	const int max_paths = vm["max-paths"].as<int>();
	const bool relax_only = vm.count("relax-only");
	const bool pre_spgm = vm.count("pre-spgm");

	// Configuration
	config conf = {
		sols_obj,
		model_config {
			.num_thread = num_thread,
			.var_select = var_select,
			.time_limit = time_limit,
			.heur_freq = heur_freq,
			.pre_cut = pre_cut,
			.full_mode = full_mode,
			.max_paths = max_paths,
			.relax_only = relax_only,
			.pre_spgm = pre_spgm
		}
	};
	cout << boolalpha << "Config:" << endl <<
		"  Number of threads: " << num_thread << endl <<
		"  Variable selection: " << var_select << endl <<
		"  Time limit: " << time_limit << endl <<
		"  Heuristic frequency: " << heur_freq << endl <<
		"  Pre-solved cuts: " << pre_cut << endl <<
		"  Full mode: " << full_mode << endl <<
		"  Max num paths: " << max_paths << endl <<
		"  Relax only: " << relax_only << endl <<
		"  Pre SPGM: " << pre_spgm << endl;

	if (vm.count("standard")) {
		report << "STANDARD:" << endl;
		if (is_multi)
			report << run_model<standard_model_multi>(env, *prob_multi, "STANDARD MODEL MULTI", conf);
		else if (is_hybrid)
			report << run_model<standard_hmodel>(env, *prob, "STANDARD HYBRID MODEL", conf);
		else
			report << run_model<standard_model>(env, *prob, "STANDARD MODEL", conf);
	}
	if (vm.count("goal")) {
		report << "STANDARD GOAL:" << endl <<
			run_model<standard_goal_model>(env, *prob, "STANDARD GOAL MODEL", conf);
	}
	if (vm.count("cscut")) {
		report << "STANDARD CSCUT:" << endl <<
			run_model<standard_cscut_model>(env, *prob, "STANDARD CSCUT MODEL", conf);
	}
	if (vm.count("vfcut")) {
		report << "STANDARD VFCUT:" << endl <<
			 run_model<standard_vfcut_model>(env, *prob, "STANDARD VFCUT MODEL", conf);
	}
	if (vm.count("lvfcut")) {
		report << "LIGHT VFCUT:" << endl <<
			run_model<light_vfcut_model>(env, *prob, "LIGHT VFCUT MODEL", conf);
	}
	if (vm.count("benders")) {
		report << "BENDERS:" << endl;
		switch (vm["benders"].as<int>())
		{
		case 0: report << run_model<benders_model_original>(env, *prob, "BENDERS MODEL 0", conf); break;
		case 1: report << run_model<benders_model_reduced>(env, *prob, "BENDERS MODEL 1", conf); break;
		case 2: report << run_model<benders_model_reduced2>(env, *prob, "BENDERS MODEL 2", conf); break;
		default:
			break;
		}

	}
	if (vm.count("valuefunc")) {
		report << "VALUEFUNC:" << endl;
		if (is_hybrid)
			report << run_model<value_func_hmodel>(env, *prob, "VALUE FUNC HYBRID MODEL", conf);
		else
			report << run_model<value_func_model>(env, *prob, "VALUE FUNC MODEL", conf);
	}
	if (vm.count("xtbenders")) {
		report << "XT-BENDERS:" << endl << run_model<benders_xt_model>(env, *prob, "XT BENDERS MODEL", conf);
	}
	if (vm.count("xybenders")) {
		report << "XY-BENDERS:" << endl << run_model<benders_xy_model>(env, *prob, "XY BENDERS MODEL", conf);
	}
	if (vm.count("xytbenders")) {
		report << "XYT-BENDERS:" << endl << run_model<benders_xyt_model>(env, *prob, "XYT BENDERS MODEL", conf);
	}
	if (vm.count("slackbr")) {
		report << "SLACKBRANCH:" << endl << run_model<slackbranch_model>(env, *prob, "SLACK-BRANCH MODEL", conf);
	}
	if (vm.count("csenum")) {
		report << "CS ENUM:" << endl << run_model<csenum>(env, *prob, "COMP-SLACK ENUM", conf);
	}
	if (vm.count("csenum-excl")) {
		report << "CS ENUM EXCL:" << endl << run_model<csenum_excl>(env, *prob, "COMP-SLACK ENUM EXCL", conf);
	}
	if (vm.count("compslack")) {
		report << "COMP SLACK:" << endl <<
			 run_model<compslack_model>(env, *prob, "COMP SLACK MODEL", conf);
	}

	if (vm.count("path-std")) {
		report << "PATH (STANDARD):" << endl <<
			run_model<path_standard_hmodel>(env, *prob, "PATH MODEL (STANDARD)", conf);
	}
	if (vm.count("path-vf")) {
		report << "PATH (VALUEFUNC):" << endl <<
			run_model<path_value_func_hmodel>(env, *prob, "PATH MODEL (VALUEFUNC)", conf);
	}
	if (vm.count("vfpath-std")) {
		report << "VFPATH (STANDARD):" << endl <<
			run_model<vfpath_standard_hmodel>(env, *prob, "VFPATH MODEL (STANDARD)", conf);
	}
	if (vm.count("vfpath-vf")) {
		report << "VFPATH (VALUEFUNC):" << endl <<
			run_model<vfpath_value_func_hmodel>(env, *prob, "VFPATH MODEL (VALUEFUNC)", conf);
	}
	if (vm.count("didi-std")) {
		report << "PATH DIDI (STANDARD):" << endl <<
			run_model<path_didi_standard_hmodel>(env, *prob, "PATH DIDI MODEL (STANDARD)", conf);
	}
	if (vm.count("didi-vf")) {
		report << "PATH DIDI (VALUEFUNC):" << endl <<
			run_model<path_didi_value_func_hmodel>(env, *prob, "PATH DIDI MODEL (VALUEFUNC)", conf);
	}
	if (vm.count("fstandard")) {
		report << "FILTERED STANDARD:" << endl <<
			run_model<filtered_standard_hmodel>(env, *prob, "FILTERED STANDARD MODEL", conf);
	}
	if (vm.count("fvaluefunc")) {
		report << "FILTERED VALUEFUNC:" << endl <<
			run_model<filtered_value_func_hmodel>(env, *prob, "FILTERED VALUE FUNC MODEL", conf);
	}
	if (vm.count("pstandard")) {
		report << "PROCESSED STANDARD:" << endl <<
			run_model<processed_standard_hmodel>(env, *prob, "PROCESSED STANDARD MODEL", conf);
	}
	if (vm.count("pvaluefunc")) {
		report << "PROCESSED VALUEFUNC:" << endl <<
			run_model<processed_value_func_hmodel>(env, *prob, "PROCESSED VALUE FUNC MODEL", conf);
	}
	if (vm.count("spgm-std")) {
		report << "SPGM STANDARD:" << endl <<
			run_model<spgm_standard_hmodel>(env, *prob, "SPGM STANDARD MODEL", conf);
	}
	if (vm.count("spgm-vf")) {
		report << "SPGM VALUEFUNC:" << endl <<
			run_model<spgm_value_func_hmodel>(env, *prob, "SPGM VALUE FUNC MODEL", conf);
	}
	if (vm.count("p-spgm-std")) {
		report << "PROCESSED-SPGM STANDARD:" << endl <<
			run_model<processed_spgm_standard_hmodel>(env, *prob, "PROCESSED-SPGM STANDARD MODEL", conf);
	}
	if (vm.count("p-spgm-vf")) {
		report << "PROCESSED-SPGM VALUEFUNC:" << endl <<
			run_model<processed_spgm_value_func_hmodel>(env, *prob, "PROCESSED-SPGM VALUE FUNC MODEL", conf);
	}
	if (vm.count("apstd-std")) {
		report << "ARC-PATH STANDARD (STANDARD):" << endl <<
			run_model<apstandard_standard_hmodel>(env, *prob, "ARC-PATH STANDARD (STANDARD) MODEL", conf);
	}
	if (vm.count("apstd-vf")) {
		report << "ARC-PATH STANDARD (VALUEFUNC):" << endl <<
			run_model<apstandard_valuefunc_hmodel>(env, *prob, "ARC-PATH STANDARD (VALUE FUNC) MODEL", conf);
	}
	if (vm.count("apvf-std")) {
		report << "ARC-PATH VALUEFUNC (STANDARD):" << endl <<
			run_model<apvaluefunc_standard_hmodel>(env, *prob, "ARC-PATH VALUE FUNC (STANDARD) MODEL", conf);
	}
	if (vm.count("apvf-vf")) {
		report << "ARC-PATH VALUEFUNC (VALUEFUNC):" << endl <<
			run_model<apvaluefunc_valuefunc_hmodel>(env, *prob, "ARC-PATH VALUE FUNC (VALUE FUNC) MODEL", conf);
	}

	if (vm.count("compose")) {
		string code = vm["compose"].as<string>();
		report << "COMPOSED (" << code << "):" << endl <<
			run_model<composed_hmodel>(env, *prob, "COMPOSED (" + code + ") MODEL", conf, code);
	}

	// Print report
	if (report.str().size()) {
		cout << "--------------------------------------" << endl;
		cout << report.str();
	}		

	// Save report file
	if (!sols_obj.is_null())
		report_obj["solutions"] = sols_obj;

	ofstream os(vm["output"].as<string>());
	os << report_obj;

	// Clean up
	env.end();
	delete prob;

	return 0;
}

bool assert_args(const vector<string>& args, int min_length) {
	if (args.size() < min_length)
		cerr << "Require " << min_length << " argument(s)" << endl;
	return args.size() >= min_length;
}

void run_routine(int index, const vector<string>& args)
{
	switch (index)
	{
	case 0: follower_solver_perftest(); break;
	case 1: follower_cplex_solver_perftest(); break;
	case 2: follower_light_solver_perftest(); break;
	case 3: follower_solver_acctest(); break;
	case 4: inverse_solver_acctest(); break;
	case 5: inverse_solver_perftest(); break;
	case 6: tolls_heuristic_perftest(); break;
	case 7: light_graph_dijkstra_acctest(); break;
	case 8: light_graph_dijkstra_perftest(); break;
	case 9: light_graph_yen_acctest(); break;
	case 10: light_graph_yen_perftest(); break;
	case 11: light_graph_toll_unique_acctest(); break;
	case 12: light_graph_toll_unique_perftest(); break;
	case 13: light_graph_price_from_src_acctest(); break;
	case 14: light_graph_price_to_dst_acctest(); break;
	case 15: light_graph_bilevel_feasible_yen_acctest(); break;
	case 16: light_graph_bilevel_feasible_yen_perftest(); break;
	case 17: light_graph_bilevel_feasible_acctest(); break;
	case 18: light_graph_bilevel_feasible_perftest(); break;
	case 19: light_graph_bilevel_feasible_stat(); break;
	case 20: path_vs_spgm_preprocessors_compare(); break;
	case 21: light_graph_bilevel_feasible_2_acctest(); break;
	case 22: light_graph_bilevel_feasible_2_perftest(); break;
	case 23: light_graph_bilevel_feasible_3_acctest(); break;
	case 24: light_graph_bilevel_feasible_3_perftest(); break;
	case 25: if (assert_args(args, 2)) data_numpaths_stats(args[0], atoi(args[1].c_str())); break;
	case 26: if (assert_args(args, 2)) data_pathenum_stats(args[0], atoi(args[1].c_str())); break;
	case 27: if (assert_args(args, 2)) data_preprocessing_stats(args[0], atoi(args[1].c_str())); break;
	case 28: if (assert_args(args, 1)) data_dimensions_stats(args[0]); break;
	case 29: if (assert_args(args, 2)) data_path_spgm_preprocessing_stats(args[0], atoi(args[1].c_str())); break;
	default:
		cerr << "Wrong routine number" << endl;
		break;
	}
}
