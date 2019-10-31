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
#include <nlohmann/json.hpp>

using namespace std;
using namespace boost;
using json = nlohmann::json;
namespace po = boost::program_options;

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
string run_model(IloEnv env, problem& prob, string model_name, json& sols_obj, int num_thread) {
	try {
		cout << "--------------------------------------" << endl;
		cout << model_name << endl;

		model_type model(env, prob);

		IloCplex cplex = model.get_cplex();
		cplex.setParam(IloCplex::ClockType, 2);
		cplex.setParam(IloCplex::Threads, num_thread);

		if (!model.solve()) {
			env.error() << "Failed to optimize LP." << endl;
			throw(-1);
		}

		env.out() << "Solution status = " << cplex.getStatus() << endl;
		env.out() << "Solution value = " << cplex.getObjValue() << endl;
		string report = model.get_report();

		json sol_obj = model.get_solution().get_json(prob);
		sol_obj["name"] = model_name;
		sols_obj.push_back(std::move(sol_obj));

		model.end();

		regex report_regex("^|(\\n)(?!$)");
		report = std::regex_replace(report, report_regex, "$1  ");
		return report;
	}
	catch (const IloException & e) {
		cerr << "Exception caught: " << e << endl;
	}
}

int main(int argc, char* argv[])
{
	const int DEFAULT_NUM_THREADS = thread::hardware_concurrency();

	// Parameter processing
	po::options_description desc("Allowed options");
	desc.add_options()
		("help,h", "display help message")
		("standard,s", "run standard model")
		("benders,b", po::value<int>(), "run benders model (arg is 0, 1, or 2)")
		("valuefunc,v", "run value function model")
		("input,i", po::value<string>(), "input problem from file")
		("output,o", po::value<string>()->default_value("report.json"), "output report file")
		("thread,t", po::value<int>()->default_value(DEFAULT_NUM_THREADS), "number of threads")
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

	// Report json object
	json report_obj, sols_obj;

	// Random generator
	auto seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
	auto random_engine = default_random_engine(seed);

	// Create a problem
	problem* prob;
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

	// Run the models
	IloEnv env;
	ostringstream report;
	const int num_thread = vm["thread"].as<int>();

	if (vm.count("standard")) {
		report << "STANDARD:" << endl << run_model<standard_model>(env, *prob, "STANDARD MODEL", sols_obj, num_thread);
	}
	if (vm.count("benders")) {
		report << "BENDERS:" << endl;
		switch (vm["benders"].as<int>())
		{
		case 0: report << run_model<benders_model_original>(env, *prob, "BENDERS MODEL 0", sols_obj, num_thread); break;
		case 1: report << run_model<benders_model_reduced>(env, *prob, "BENDERS MODEL 1", sols_obj, num_thread); break;
		case 2: report << run_model<benders_model_reduced2>(env, *prob, "BENDERS MODEL 2", sols_obj, num_thread); break;
		default:
			break;
		}

	}
	if (vm.count("valuefunc")) {
		report << "VALUEFUNC:" << endl << run_model<value_func_model>(env, *prob, "VALUE FUNC MODEL", sols_obj, num_thread);
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
