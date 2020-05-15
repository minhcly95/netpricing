#include "../graph/light_graph.h"
#include "../problem_generator.h"

#include "../hybrid/preprocessors/path_preprocessor.h"
#include "../hybrid/preprocessors/spgm_preprocessor.h"

#include <chrono>

using namespace std;

void path_vs_spgm_preprocessors_compare() {
	using path = vector<int>;

	cout << "Path preprocessor vs SPGM preprocessor (5 x 12 grid, K = 40) statistics..." << endl;

	const int SAMPLES = 1000;
	const int MAX_PATHS = 500;

	auto seed = chrono::high_resolution_clock::now().time_since_epoch().count();
	auto random_engine = default_random_engine(seed);

	vector<problem> probs;

	LOOP(i, SAMPLES) {
		probs.emplace_back(random_grid_problem(5, 12, 40, 0.2, random_engine));
	}

	LOOP(i, SAMPLES) {
		problem& prob = probs[i];
		light_graph lgraph(prob.graph);

		const char* TAB = "\t";

		LOOP(k, prob.commodities.size()) {
			commodity& comm = prob.commodities[k];
			vector<path> ps = lgraph.bilevel_feasible_paths(comm.origin, comm.destination, MAX_PATHS + 1);

			cout << ps.size() << TAB;

			if (ps.size() <= MAX_PATHS) {
				path_preprocessor pproc(ps);

				std::cout.setstate(std::ios_base::failbit);
				auto info1 = pproc.preprocess(prob, k);
				std::cout.clear();

				cout << info1.V.size() << TAB << info1.A.size() << TAB << info1.A1.size() << TAB;
			}
			else {
				cout << "-1" << TAB << "-1" << TAB << "-1" << TAB;
			}

			spgm_preprocessor sproc;

			std::cout.setstate(std::ios_base::failbit);
			auto info2 = sproc.preprocess(prob, k);
			std::cout.clear();

			cout << info2.V.size() << TAB << info2.A.size() << TAB << info2.A1.size() << endl;
		}
	}
}