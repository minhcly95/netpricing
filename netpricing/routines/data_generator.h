#pragma once

#include <vector>
#include <random>
#include "../typedef.h"
#include "../macros.h"

template <class solver_type, class random_type>
std::vector<std::vector<cost_type>> generate_tolls_data(const solver_type& solver, int samples, random_type& random_engine) {
	using namespace std;

	vector<vector<cost_type>> tolls_data(samples, vector<cost_type>(solver.A1));

	LOOP(a, solver.A1) {
		uniform_real_distribution<> dist(0, solver.prob.big_n[a]);
		LOOP(i, samples) {
			tolls_data[i][a] = dist(random_engine);
		}
	}

	return tolls_data;
}
