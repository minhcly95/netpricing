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

string run_standard_model(IloEnv env, problem& prob);
string run_benders_model0(IloEnv env, problem& prob);
string run_benders_model1(IloEnv env, problem& prob);
string run_benders_model2(IloEnv env, problem& prob);
string run_value_func_model(IloEnv env, problem& prob);

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
		report << "STANDARD:" << endl << run_standard_model(env, *prob);
	}
	if (vm.count("benders")) {
		report << "BENDERS:" << endl;
		switch (vm["benders"].as<int>())
		{
		case 0: report << run_benders_model0(env, *prob); break;
		case 1: report << run_benders_model1(env, *prob); break;
		case 2: report << run_benders_model2(env, *prob); break;
		default:
			break;
		}
		
	}
	if (vm.count("valuefunc")) {
		report << "VALUEFUNC:" << endl << run_value_func_model(env, *prob);
	}

	// Print report
	cout << "--------------------------------------" << endl;
	cout << report.str();

	delete prob;
	env.end();

	return 0;
}

template <class model_type>
pair<model_type, IloCplex> run_model_with_callback(IloEnv env, problem& prob) {
	model_type model(env, prob);

	IloCplex cplex(model.cplex_model);

	auto cb = model.attach_callback(cplex);
	cplex.setParam(IloCplex::Threads, 8);
	cplex.setParam(IloCplex::ClockType, 2);
	cplex.resetTime();

	if (!cplex.solve()) {
		env.error() << "Failed to optimize LP." << endl;
		throw(-1);
	}

	env.out() << "Solution status = " << cplex.getStatus() << endl;
	env.out() << "Solution value = " << cplex.getObjValue() << endl;

	cb.end();

	return make_pair(model, cplex);
}

string run_standard_model(IloEnv env, problem& prob) {
	try {
		cout << "--------------------------------------" << endl;
		cout << "STANDARD MODEL" << endl;

		standard_model model(env, prob);

		IloCplex cplex(model.cplex_model);
		cplex.setParam(IloCplex::ClockType, 2);
		cplex.resetTime();

		if (!cplex.solve()) {
			env.error() << "Failed to optimize LP." << endl;
			throw(-1);
		}

		env.out() << "Solution status = " << cplex.getStatus() << endl;
		env.out() << "Solution value = " << cplex.getObjValue() << endl;
		double value = cplex.getObjValue();

		ostringstream ss;
		ss << "OBJ: " << value << endl <<
			"TIME: " << cplex.getTime() << " s" << endl;

		return ss.str();
	}
	catch (const IloException & e) {
		cerr << "Exception caught: " << e << endl;
	}
}

string run_benders_model0(IloEnv env, problem& prob) {
	try {
		cout << endl << "--------------------------------------" << endl;
		cout << "BENDERS MODEL 0" << endl;

		auto model_pair = run_model_with_callback<benders_model_original>(env, prob);
		double value = model_pair.second.getObjValue();
		auto model = model_pair.first;

		ostringstream ss;
		ss << "OBJ: " << value << endl <<
			"TIME: " << model_pair.second.getTime() << " s" <<
			"    Sep " << model.separate_time << " s" <<
			"    Avg " << (model.separate_time * 1000 / model.separate_count) << " ms" <<
			"    Sub " << (model.subprob_time * 100 / model.separate_time) << "%" << endl <<
			"SEP: Total " << model.separate_count << endl;

		return ss.str();
	}
	catch (const IloException & e) {
		cerr << "Exception caught: " << e << endl;
	}
}

string run_benders_model1(IloEnv env, problem& prob) {
	try {
		cout << endl << "--------------------------------------" << endl;
		cout << "BENDERS MODEL 1" << endl;

		auto model_pair = run_model_with_callback<benders_model_reduced>(env, prob);
		double value = model_pair.second.getObjValue();
		auto model = model_pair.first;

		ostringstream ss;
		ss << "OBJ: " << value << endl <<
			"TIME: " << model_pair.second.getTime() << " s" <<
			"    Sep " << model.separate_time << " s" <<
			"    Avg " << (model.separate_time * 1000 / model.separate_count) << " ms" <<
			"    Sub1 " << (model.subprob1_time * 100 / model.separate_time) << "%" <<
			"    Sub3 " << (model.subprob3_time * 100 / model.separate_time) << "%" << endl <<
			"SEP: Total " << model.separate_count <<
			"    F " << model.flow_cut_count <<
			"    T " << model.toll_cut_count <<
			"    O " << model.opt_cut_count << endl;

		return ss.str();
	}
	catch (const IloException & e) {
		cerr << "Exception caught: " << e << endl;
	}
}

string run_benders_model2(IloEnv env, problem& prob) {
	try {
		cout << endl << "--------------------------------------" << endl;
		cout << "BENDERS MODEL 2" << endl;

		auto model_pair = run_model_with_callback<benders_model_reduced2>(env, prob);
		double value = model_pair.second.getObjValue();
		auto model = model_pair.first;

		ostringstream ss;
		ss << "OBJ: " << value << endl <<
			"TIME: " << model_pair.second.getTime() << " s" <<
			"    Sep " << model.separate_time << " s" <<
			"    Avg " << (model.separate_time * 1000 / model.separate_count) << " ms" <<
			"    Sub1 " << (model.subprob1_time * 100 / model.separate_time) << "%" <<
			"    Sub2 " << (model.subprob2_time * 100 / model.separate_time) << "%" <<
			"    Sub3 " << (model.subprob3_time * 100 / model.separate_time) << "%" << endl <<
			"SEP: Total " << model.separate_count <<
			"    F " << model.flow_cut_count <<
			"    P " << model.path_cut_count <<
			"    T " << model.toll_cut_count <<
			"    O " << model.opt_cut_count << endl;

		return ss.str();
	}
	catch (const IloException & e) {
		cerr << "Exception caught: " << e << endl;
	}
}

string run_value_func_model(IloEnv env, problem& prob) {
	try {
		cout << endl << "--------------------------------------" << endl;
		cout << "VALUE FUNC MODEL" << endl;

		auto model_pair = run_model_with_callback<value_func_model>(env, prob);
		double value = model_pair.second.getObjValue();
		auto model = model_pair.first;

		ostringstream ss;
		ss << "OBJ: " << value << endl <<
			"TIME: " << model_pair.second.getTime() << " s" <<
			"    Sep " << model.separate_time << " s" <<
			"    Avg " << (model.separate_time * 1000 / model.separate_count) << " ms" <<
			"    Sub " << (model.subprob_time * 100 / model.separate_time) << "%" << endl <<
			"SEP: Total " << model.separate_count << endl;

		return ss.str();
	}
	catch (const IloException & e) {
		cerr << "Exception caught: " << e << endl;
	}
}
