#include <iostream>
#include <string>
#include <regex>
#include <experimental/filesystem>

#include "../macros.h"
#include "../problem.h"
#include "../graph/light_graph.h"
#include "../hybrid/preprocessors/path_preprocessor.h"
#include "../hybrid/preprocessors/spgm_preprocessor.h"

using namespace std;
namespace fs = std::experimental::filesystem;

void data_numpaths_stats(string prefix, int numpaths) {
	regex fileregex("^" + prefix + "\\S*\\.json$");

	for (auto& entry : fs::directory_iterator(".")) {
		string filename = string(entry.path().filename());
		if (!regex_match(filename, fileregex))
			continue;

		problem prob = problem::read_from_json(filename)[0];
		light_graph lgraph(prob.graph);

		LOOP(k, prob.commodities.size()) {
			commodity& comm = prob.commodities[k];
			auto ps = lgraph.bilevel_feasible_paths_2(comm.origin, comm.destination, numpaths + 1);
			if (ps.size() <= numpaths)
				cout << ps.size() << endl;
			else
				cout << numpaths + 1 << endl;
		}
	}
}

void data_pathenum_stats(string prefix, int numpaths) {
	regex fileregex("^" + prefix + "\\S*\\.json$");

	for (auto& entry : fs::directory_iterator(".")) {
		string filename = string(entry.path().filename());
		if (!regex_match(filename, fileregex))
			continue;

		problem prob = problem::read_from_json(filename)[0];
		light_graph lgraph(prob.graph);

		LOOP(k, prob.commodities.size()) {
			commodity& comm = prob.commodities[k];
			auto ps = lgraph.bilevel_feasible_paths_2(comm.origin, comm.destination, numpaths + 1, false);
			auto rs = lgraph.filter_bilevel_feasible(ps);

			cout << ps.size() << "\t" << rs.size() << endl;
		}
	}
}

void data_preprocessing_stats(string prefix, int numpaths) {
	regex fileregex("^" + prefix + "\\S*\\.json$");

	for (auto& entry : fs::directory_iterator(".")) {
		string filename = string(entry.path().filename());
		if (!regex_match(filename, fileregex))
			continue;

		problem prob = problem::read_from_json(filename)[0];
		light_graph lgraph(prob.graph);

		const char* TAB = "\t";

		LOOP(k, prob.commodities.size()) {
			commodity& comm = prob.commodities[k];
			auto ps = lgraph.bilevel_feasible_paths_2(comm.origin, comm.destination, numpaths + 1);

			cout << ps.size() << TAB;

			if (ps.size() <= numpaths) {
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

void data_dimensions_stats(string prefix) {
	regex fileregex("^" + prefix + "\\S*\\.json$");

	for (auto& entry : fs::directory_iterator(".")) {
		string filename = string(entry.path().filename());
		if (!regex_match(filename, fileregex))
			continue;

		problem prob = problem::read_from_json(filename)[0];
		light_graph lgraph(prob.graph);

		cout << lgraph.V << "\t" <<
			lgraph.Eall.size() << endl;
	}
}
