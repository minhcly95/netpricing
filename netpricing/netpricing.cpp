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
	int num_thread;
	int var_select;
	int time_limit;
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

template <class model_type>
string run_model(IloEnv& env, typename model_type::problem_type& prob, string model_name, config& conf) {
	//try {
		cout << "--------------------------------------" << endl;
		cout << model_name << endl;

		model_type model(env, prob);
		model.config(model_config{
			.num_thread = conf.num_thread,
			.var_select = conf.var_select,
			.time_limit = conf.time_limit
					 });

		if (!model.solve()) {
			env.error() << "Failed to optimize LP." << endl;
			throw(-1);
		}

		/*env.out() << "Solution status = " << cplex.getStatus() << endl;
		env.out() << "Solution value = " << cplex.getObjValue() << endl;*/
		string report = model.get_report();

		json sol_obj = model.get_solution().get_json(prob);
		sol_obj["name"] = model_name;
		conf.sols_obj.push_back(std::move(sol_obj));

		model.end();

		regex report_regex("^|(\\n)(?!$)");
		report = std::regex_replace(report, report_regex, "$1  ");
		return report;
	//}
	//catch (const IloException & e) {
	//	cerr << "Exception caught: " << e << endl;
	//}
}

void run_routine(int index);

int main(int argc, char* argv[])
{
	const int DEFAULT_NUM_THREADS = thread::hardware_concurrency();

	// Parameter processing
	po::options_description desc("Allowed options");
	desc.add_options()
		("help,h", "display help message")
		("routine,r", po::value<int>(), "run special routines")
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
		("multi", "run multi-graph version")
		("input,i", po::value<string>(), "input problem from file")
		("output,o", po::value<string>()->default_value("report.json"), "output report file")
		("thread,t", po::value<int>()->default_value(DEFAULT_NUM_THREADS), "number of threads")
		("var-select,v", po::value<int>()->default_value(IloCplex::DefaultVarSel), "variable selection strategy")
		("time,T", po::value<int>()->default_value(0), "time limit (0 = no limit)")
		("nodes,n", po::value<int>()->default_value(10), "number of nodes in the random problem")
		("arcs,a", po::value<int>()->default_value(20), "number of arcs in the random problem")
		("commodities,k", po::value<int>()->default_value(5), "number of commodities in the random problem")
		("toll-proportion,p", po::value<float>()->default_value(0.2f), "toll proportion in the random problem")
		("grid", po::value<grid_params>()->multitoken(), "create a grid problem")
		("delaunay", po::value<int>(), "create a Delaunay graph problem")
		("voronoi", po::value<int>(), "create a Voronoi graph problem");

	po::variables_map vm;

	try {
		po::store(po::parse_command_line(argc, argv, desc), vm);
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
		run_routine(vm["routine"].as<int>());
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

		cout << "GRID NETWORK created:" << endl;
		cout << "    width  = " << params.width << endl;
		cout << "    height = " << params.height << endl;
		cout << "    k      = " << k << endl;
		cout << "    p      = " << p << endl;
	}
	else if (vm.count("delaunay")) {
		int n = vm["delaunay"].as<int>();
		int k = vm["commodities"].as<int>();
		float p = vm["toll-proportion"].as<float>();
		prob = new problem(random_delaunay_problem(n, k, p, random_engine));

		cout << "DELAUNAY NETWORK created:" << endl;
		cout << "    n = " << n << endl;
		cout << "    k = " << k << endl;
		cout << "    p = " << p << endl;
	}
	else if (vm.count("voronoi")) {
		int num_seeds = vm["voronoi"].as<int>();
		int k = vm["commodities"].as<int>();
		float p = vm["toll-proportion"].as<float>();
		prob = new problem(random_voronoi_problem(num_seeds, k, p, random_engine));
		int n = num_vertices(prob->graph);

		cout << "VORONOI NETWORK created:" << endl;
		cout << "    num_seeds = " << num_seeds << endl;
		cout << "    n         = " << n << endl;
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

	// Run the models
	IloEnv env;
	ostringstream report;
	const int num_thread = vm["thread"].as<int>();
	const int var_select = vm["var-select"].as<int>();
	const int time_limit = vm["time"].as<int>();

	// Configuration
	config conf = {
		sols_obj,
		num_thread,
		var_select,
		time_limit
	};
	cout << "Config:" << endl <<
			"  Number of threads: " << num_thread << endl <<
			"  Variable selection: " << var_select << endl <<
			"  Time limit: " << time_limit << endl;

	if (vm.count("standard")) {
		report << "STANDARD:" << endl <<
			(is_multi ?
			 run_model<standard_model_multi>(env, *prob_multi, "STANDARD MODEL MULTI", conf) :
			 run_model<standard_model>(env, *prob, "STANDARD MODEL", conf));
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
		report << "VALUEFUNC:" << endl << run_model<value_func_model>(env, *prob, "VALUE FUNC MODEL", conf);
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

void run_routine(int index)
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
	default:
		cerr << "Wrong routine number" << endl;
		break;
	}
}
