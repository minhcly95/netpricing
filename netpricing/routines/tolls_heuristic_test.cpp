#include "../heuristics/tolls_heuristic.h"
#include "../problem_generator.h"
#include "data_generator.h"

#include <chrono>

using namespace std;

void tolls_heuristic_perftest() {
	cout << "Tolls heuristic performance test..." << endl;

	const int SAMPLES = 1000;
	const int REPEAT = 1;
	const int TOTAL = SAMPLES * REPEAT;

	auto seed = chrono::high_resolution_clock::now().time_since_epoch().count();
	auto random_engine = default_random_engine(seed);

	problem prob(random_grid_problem(5, 12, 40, 0.2, random_engine));
	IloEnv env;

	tolls_heuristic heur(env, prob);

	vector<vector<cost_type>> tolls_data = generate_tolls_data(heur.solver_f, SAMPLES, random_engine);

	auto start = chrono::high_resolution_clock::now();

	LOOP(i, SAMPLES) {
		LOOP(j, REPEAT) {
			heur.solve(tolls_data[i]);
		}
	}

	auto end = chrono::high_resolution_clock::now();
	double time = chrono::duration<double>(end - start).count();

	cout << "Outer measurement: " << time * 1000 / TOTAL << " ms" << endl;
	cout << "Inner measurement: " << heur.time * 1000 / TOTAL << " ms" << endl;
}
