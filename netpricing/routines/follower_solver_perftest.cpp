#include "../follower_solver.h"
#include "../follower_cplex_solver.h"
#include "../problem_generator.h"

#include <chrono>

using namespace std;

template <class solver_type, class random_type>
vector<vector<cost_type>> generate_tolls_data(const solver_type& solver, int samples, random_type& random_engine) {
	vector<vector<cost_type>> tolls_data(samples, vector<cost_type>(solver.A1));

	LOOP(a, solver.A1) {
		uniform_real_distribution<> dist(0, solver.prob.big_n[a]);
		LOOP(i, samples) {
			tolls_data[i][a] = dist(random_engine);
		}
	}

	return tolls_data;
}

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

	const int SAMPLES = 100;
	const int REPEAT = 10;
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
