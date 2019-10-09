// netpricing.cpp : Defines the entry point for the application.
//

#include "netpricing.h"
#include <chrono>
#include <string>
#include <sstream>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/program_options.hpp>

using namespace std;
using namespace boost;
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
string run_model(IloEnv env, problem& prob, string model_name) {
	try {
		cout << "--------------------------------------" << endl;
		cout << model_name << endl;

		model_type model(env, prob);

		IloCplex cplex = model.get_cplex();
		cplex.setParam(IloCplex::ClockType, 2);
		cplex.setParam(IloCplex::Threads, 8);

		if (!model.solve()) {
			env.error() << "Failed to optimize LP." << endl;
			throw(-1);
		}

		env.out() << "Solution status = " << cplex.getStatus() << endl;
		env.out() << "Solution value = " << cplex.getObjValue() << endl;
		string report = model.get_report();

		model.end();
		return report;
	}
	catch (const IloException & e) {
		cerr << "Exception caught: " << e << endl;
	}
}

int main(int argc, char* argv[])
{
	// Parameter processing
	po::options_description desc("Allowed options");
	desc.add_options()
		("help,h", "display help message")
		("standard,s", "run standard model")
		("benders,b", po::value<int>(), "run benders model (arg is 0, 1, or 2)")
		("valuefunc,v", "run value function model")
		("file,f", po::value<string>(), "input problem from file")
		("nodes,n", po::value<int>()->default_value(10), "number of nodes in the random problem")
		("arcs,a", po::value<int>()->default_value(20), "number of arcs in the random problem")
		("commodities,k", po::value<int>()->default_value(5), "number of commodities in the random problem")
		("toll-proportion,t", po::value<float>()->default_value(0.2f), "toll proportion in the random problem");

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm);

	// Help
	if (vm.count("help")) {
		cout << desc << endl;
		return 0;
	}

	// Create a problem
	problem* prob;
	if (vm.count("file")) {
		prob = new problem(problem::read_from_json(vm["file"].as<string>())[0]);
	}
	else {
		auto seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
		auto random_engine = default_random_engine(seed);
		int n = vm["nodes"].as<int>();
		int a = vm["arcs"].as<int>();
		int k = vm["commodities"].as<int>();
		float t = vm["toll-proportion"].as<float>();
		prob = new problem(random_problem(n, a, k, t, random_engine));
	}
	prob->write_to_json("current.json");

	// Run the models
	IloEnv env;
	ostringstream report;

	if (vm.count("standard")) {
		report << "STANDARD:" << endl << run_model<standard_model>(env, *prob, "STANDARD MODEL");
	}
	if (vm.count("benders")) {
		report << "BENDERS:" << endl;
		switch (vm["benders"].as<int>())
		{
		case 0: report << run_model<benders_model_original>(env, *prob, "BENDERS MODEL 0"); break;
		case 1: report << run_model<benders_model_reduced>(env, *prob, "BENDERS MODEL 1"); break;
		case 2: report << run_model<benders_model_reduced2>(env, *prob, "BENDERS MODEL 2"); break;
		default:
			break;
		}
		
	}
	if (vm.count("valuefunc")) {
		report << "VALUEFUNC:" << endl << run_model<value_func_model>(env, *prob, "VALUE FUNC MODEL");
	}

	// Print report
	cout << "--------------------------------------" << endl;
	cout << report.str();

	delete prob;
	env.end();

	return 0;
}
