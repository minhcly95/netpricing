// netpricing.cpp : Defines the entry point for the application.
//

#include "netpricing.h"

using namespace std;
using namespace boost;
int main()
{
	//problem prob = random_problem(10, 20, 5, 0.5f);
	problem prob = std::move(problem::read_from_json("../../../../resources/problems/g10-5.json")[0]);

	IloEnv env;

	try {
		benders_model_original model(env, prob);

		IloCplex cplex(model.cplex_model);
		auto cb = model.attach_callback(cplex);
		//cplex.setParam(IloCplex::PreInd, IloFalse);
		//cplex.setParam(IloCplex::Threads, 1);

		if (!cplex.solve()) {
			env.error() << "Failed to optimize LP." << endl;
			throw(-1);
		}

		env.out() << "Solution status = " << cplex.getStatus() << endl;
		env.out() << "Solution value = " << cplex.getObjValue() << endl;

		cb.end();
	}
	catch (const IloException& e) {
		cerr << "Exception caught: " << e << endl;
	}
	env.end();

	return 0;
}
