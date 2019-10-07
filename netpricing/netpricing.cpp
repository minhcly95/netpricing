// netpricing.cpp : Defines the entry point for the application.
//

#include "netpricing.h"
#include <chrono>

using namespace std;
using namespace boost;

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

int main()
{
	auto seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
	auto random_engine = default_random_engine(seed);
	problem prob = random_problem(50, 200, 20, 0.2f, random_engine);
	//problem prob = std::move(problem::read_from_json("../../../../resources/problems/g10-4.json")[0]);

	prob.write_to_json("current.lp");

	IloEnv env;

	try {
		cout << "--------------------------------------" << endl;
		cout << "STANDARD MODEL" << endl;

		standard_model smodel(env, prob);
		IloCplex scplex(smodel.cplex_model);

		if (!scplex.solve()) {
			env.error() << "Failed to optimize LP." << endl;
			throw(-1);
		}

		env.out() << "Solution status = " << scplex.getStatus() << endl;
		env.out() << "Solution value = " << scplex.getObjValue() << endl;
		double svalue = scplex.getObjValue();

		cout << endl << "--------------------------------------" << endl;
		cout << "BENDERS MODEL" << endl;

		benders_model_reduced2 bmodel(env, prob);
		//print_src_dst(prob, bmodel);

		IloCplex bcplex(bmodel.cplex_model);
		auto cb = bmodel.attach_callback(bcplex);
		//cplex.setParam(IloCplex::PreInd, IloFalse);
		//cplex.setParam(IloCplex::Threads, 1);

		if (!bcplex.solve()) {
			env.error() << "Failed to optimize LP." << endl;
			throw(-1);
		}

		env.out() << "Solution status = " << bcplex.getStatus() << endl;
		env.out() << "Solution value = " << bcplex.getObjValue() << endl;
		double bvalue = bcplex.getObjValue();

		cb.end();

		cout << endl << "--------------------------------------" << endl;
		cout << "S: " << svalue << " - B: " << bvalue << endl;
		cout << "SEP: Total " << bmodel.separate_count <<
			"    F " << bmodel.flow_cut_count <<
			"    P " << bmodel.path_cut_count <<
			"    T " << bmodel.toll_cut_count <<
			"    O " << bmodel.opt_cut_count << endl <<
			"TIME: Total " << bmodel.separate_time << " s" <<
			"    Avg " << (bmodel.separate_time * 1000 / bmodel.separate_count) << " ms" <<
			"    Sub1 " << (bmodel.subprob1_time * 100 / bmodel.separate_time) << "%" <<
			"    Sub2 " << (bmodel.subprob2_time * 100 / bmodel.separate_time) << "%" <<
			"    Sub3 " << (bmodel.subprob3_time * 100 / bmodel.separate_time) << "%" << endl;
	}
	catch (const IloException& e) {
		cerr << "Exception caught: " << e << endl;
	}
	env.end();

	return 0;
}
