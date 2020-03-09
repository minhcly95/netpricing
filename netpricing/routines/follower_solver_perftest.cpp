#include "../utilities/follower_solver.h"
#include "../utilities/follower_cplex_solver.h"
#include "../utilities/follower_light_solver.h"
#include "../problem_generator.h"
#include "data_generator.h"

#include <chrono>

using namespace std;

void follower_solver_perftest() {
	cout << "Follower solver performance test..." << endl;

	const int SAMPLES = 100;
	const int REPEAT = 10;
	const int TOTAL = SAMPLES * REPEAT;

	auto seed = chrono::high_resolution_clock::now().time_since_epoch().count();
	auto random_engine = default_random_engine(seed);

	problem prob(random_grid_problem(5, 12, 40, 0.2, random_engine));
	follower_solver solver(prob);
	vector<vector<cost_type>> tolls_data = generate_tolls_data(solver, SAMPLES, random_engine);

	auto start = chrono::high_resolution_clock::now();

	LOOP(i, SAMPLES) {
		LOOP(j, REPEAT) {
			solver.solve(tolls_data[i]);
		}
	}

	auto end = chrono::high_resolution_clock::now();
	double time = chrono::duration<double>(end - start).count();

	cout << "Outer measurement: " << time * 1000 / TOTAL << " ms" << endl;
	cout << "Inner measurement: " << solver.time * 1000 / TOTAL << " ms" << endl;
}

void follower_cplex_solver_perftest() {
	cout << "Follower CPLEX solver performance test..." << endl;

	const int SAMPLES = 1000;
	const int REPEAT = 1;
	const int TOTAL = SAMPLES * REPEAT;

	auto seed = chrono::high_resolution_clock::now().time_since_epoch().count();
	auto random_engine = default_random_engine(seed);
	IloEnv env;

	problem prob(random_grid_problem(5, 12, 40, 0.2, random_engine));
	follower_cplex_solver solver(env, prob);
	vector<vector<cost_type>> tolls_data = generate_tolls_data(solver, SAMPLES, random_engine);

	auto start = chrono::high_resolution_clock::now();

	LOOP(i, SAMPLES) {
		LOOP(j, REPEAT) {
			solver.solve(tolls_data[i]);
		}
	}

	auto end = chrono::high_resolution_clock::now();
	double time = chrono::duration<double>(end - start).count();

	cout << "Outer measurement: " << time * 1000 / TOTAL << " ms" << endl;
	cout << "Inner measurement: " << solver.time * 1000 / TOTAL << " ms" << endl;
}

void follower_light_solver_perftest() {
	cout << "Follower light solver performance test..." << endl;

	const int SAMPLES = 100;
	const int REPEAT = 100;
	const int TOTAL = SAMPLES * REPEAT;

	auto seed = chrono::high_resolution_clock::now().time_since_epoch().count();
	auto random_engine = default_random_engine(seed);

	problem prob(random_grid_problem(5, 12, 40, 0.2, random_engine));
	follower_light_solver solver(prob);
	vector<vector<cost_type>> tolls_data = generate_tolls_data(solver, SAMPLES, random_engine);

	auto start = chrono::high_resolution_clock::now();

	LOOP(i, SAMPLES) {
		LOOP(j, REPEAT) {
			solver.solve(tolls_data[i]);
		}
	}

	auto end = chrono::high_resolution_clock::now();
	double time = chrono::duration<double>(end - start).count();

	cout << "Outer measurement: " << time * 1000 / TOTAL << " ms" << endl;
	cout << "Inner measurement: " << solver.time * 1000 / TOTAL << " ms" << endl;
}

void follower_solver_acctest() {
	cout << "Follower solver accuracy test..." << endl;

	const int SAMPLES = 1000;

	auto seed = chrono::high_resolution_clock::now().time_since_epoch().count();
	auto random_engine = default_random_engine(seed);

	problem prob(random_grid_problem(5, 12, 40, 0.2, random_engine));
	IloEnv env;

	follower_solver solver1(prob);
	follower_cplex_solver solver2(env, prob);
	follower_light_solver solver3(prob);

	vector<vector<cost_type>> tolls_data = generate_tolls_data(solver1, SAMPLES, random_engine);

	auto start = chrono::high_resolution_clock::now();

	LOOP(i, SAMPLES) {
		auto paths1 = solver1.solve(tolls_data[i]);
		auto paths2 = solver2.solve(tolls_data[i]);
		auto paths3 = solver3.solve(tolls_data[i]);

		/*vector<cost_type> cost1, cost2, cost3;
		transform(paths1.begin(), paths1.end(), back_inserter(cost1), [&](const auto& p) { return solver1.get_cost(p, tolls_data[i]); });
		transform(paths2.begin(), paths2.end(), back_inserter(cost2), [&](const auto& p) { return solver2.get_cost(p, tolls_data[i]); });
		transform(paths3.begin(), paths3.end(), back_inserter(cost2), [&](const auto& p) { return solver3.get_cost(p, tolls_data[i]); });*/

		if (paths1 != paths2) {
			LOOP(k, solver1.K) {
				if (paths1[k] != paths2[k])
					cerr << "CPLEX solver gives wrong result at commodity " << k << endl;
			}
			return;
		}
		if (paths1 != paths3) {
			LOOP(k, solver1.K) {
				if (paths1[k] != paths3[k])
					cerr << "Light solver gives wrong result at commodity " << k << endl;
			}
			return;
		}
	}
	cout << "Follower solvers produced accurate results" << endl;
}
