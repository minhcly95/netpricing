// netpricing.cpp : Defines the entry point for the application.
//

#include "netpricing.h"

using namespace std;
using namespace boost;

int main()
{
	//problem prob = random_problem(10, 20, 5, 0.5f);
	problem prob = std::move(problem::read_from_json("../../../../resources/problems/g10-4.json")[0]);

	IloEnv env;

	standard_model model(env, prob);

	IloCplex cplex(model.cplex_model);
	if (!cplex.solve()) {
		env.error() << "Failed to optimize LP." << endl;
		throw(-1);
	}

	env.out() << "Solution status = " << cplex.getStatus() << endl;
	env.out() << "Solution value = " << cplex.getObjValue() << endl;

	env.end();

	return 0;
}
