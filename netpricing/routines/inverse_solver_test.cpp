#include "../utilities/follower_solver.h"
#include "../utilities/inverse_solver.h"
#include "../problem_generator.h"
#include "data_generator.h"

#include <chrono>

using namespace std;

void inverse_solver_acctest() {
	cout << "Inverse solver accuracy test..." << endl;

	const int SAMPLES = 1000;

	auto seed = chrono::high_resolution_clock::now().time_since_epoch().count();
	auto random_engine = default_random_engine(seed);

	problem prob(random_grid_problem(5, 12, 40, 0.2, random_engine));
	IloEnv env;

	inverse_solver solveri(env, prob);
	follower_solver solverf(prob);

	vector<vector<cost_type>> tolls_data = generate_tolls_data(solveri, SAMPLES, random_engine);

	auto start = chrono::high_resolution_clock::now();

	LOOP(i, SAMPLES) {
		auto paths1 = solverf.solve(tolls_data[i]);
		auto tolls = solveri.solve(paths1);
		auto paths2 = solverf.solve(tolls);

		vector<cost_type> cost1, cost2;
		transform(paths1.begin(), paths1.end(), back_inserter(cost1), [&](const auto& p) { return solverf.get_cost(p, tolls); });
		transform(paths2.begin(), paths2.end(), back_inserter(cost2), [&](const auto& p) { return solverf.get_cost(p, tolls); });

		LOOP(k, solveri.K) {
			if (abs(cost1[k] - cost2[k]) > 0.01) {
				cerr << "Test failed at commodity " << k << endl;
				return;
			}
		}
	}
	cout << "Inverse solver produced accurate results" << endl;
}

void inverse_solver_perftest() {
	cout << "Inverse solver performance test..." << endl;

	const int SAMPLES = 1000;
	const int REPEAT = 1;
	const int TOTAL = SAMPLES * REPEAT;

	auto seed = chrono::high_resolution_clock::now().time_since_epoch().count();
	auto random_engine = default_random_engine(seed);

	problem prob(random_grid_problem(5, 12, 40, 0.2, random_engine));
	IloEnv env;

	inverse_solver solver(env, prob);
	follower_solver solverf(prob);

	vector<vector<cost_type>> tolls_data = generate_tolls_data(solver, SAMPLES, random_engine);
	vector<vector<vector<int>>> paths_data;
	transform(tolls_data.begin(), tolls_data.end(), back_inserter(paths_data), [&](const auto& t) { return solverf.solve(t); });

	auto start = chrono::high_resolution_clock::now();

	LOOP(i, SAMPLES) {
		LOOP(j, REPEAT) {
			solver.solve(paths_data[i]);
		}
	}

	auto end = chrono::high_resolution_clock::now();
	double time = chrono::duration<double>(end - start).count();

	cout << "Outer measurement: " << time * 1000 / TOTAL << " ms" << endl;
	cout << "Inner measurement: " << solver.time * 1000 / TOTAL << " ms" << endl;
}
