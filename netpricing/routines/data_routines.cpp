#include <iostream>
#include <string>
#include <regex>
#include <experimental/filesystem>

#include "../macros.h"
#include "../problem.h"
#include "../graph/light_graph.h"

using namespace std;
namespace fs = std::experimental::filesystem;

void data_numpaths_stats(string prefix) {
	const int MAX_PATHS = 1000;

	regex fileregex("^" + prefix + "\\S*\\.json$");

	for (auto& entry : fs::directory_iterator(".")) {
		string filename = string(entry.path().filename());
		if (!regex_match(filename, fileregex))
			continue;

		problem prob = problem::read_from_json(filename)[0];
		light_graph lgraph(prob.graph);

		LOOP(k, prob.commodities.size()) {
			commodity& comm = prob.commodities[k];
			auto ps = lgraph.bilevel_feasible_paths_2(comm.origin, comm.destination, MAX_PATHS + 1);
			if (ps.size() <= MAX_PATHS)
				cout << ps.size() << endl;
			else
				cout << MAX_PATHS + 1 << endl;
		}
	}
}