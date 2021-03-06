#include "composed_hmodel.h"

#include "../macros.h"
#include "formulations/null_formulation.h"
#include "formulations/general_formulation.h"
#include "formulations/standard_formulation.h"
#include "preprocessors/spgm_preprocessor.h"
#include "formulations/sstd_formulation.h"

#include <iostream>
#include <sstream>

using namespace std;

map<string, string> composed_hmodel::VALID_CODES = {
	{"std", "Standard"},
	{"apstd", "Arc-Path Standard"},
	{"vf", "Value Function"},
	{"pvf", "Path Value Function"},
	{"cs1", "Complementary Slackness - Direct"},
	{"apcs1", "Arc-Path Complementary Slackness - Direct"},
	{"vfcs1", "Value Function Complementary Slackness - Direct"},
	{"pcs1", "Path Complementary Slackness - Direct"},
	{"cs2", "Complementary Slackness - Substitution"},
	{"apcs2", "Arc-Path Complementary Slackness - Substitution"},
	{"vfcs2", "Value Function Complementary Slackness - Substitution"},
	{"pcs2", "Path Complementary Slackness - Substitution"},
	{"spgm", "SPGM-processed Standard"},
	{"sstd", "SPGM-optional Standard"}
};
map<string, string> composed_hmodel::VALID_FALLBACK = {
	{"ustd", "Unprocessed Standard"},
	{"spgm", "SPGM-processed Standard"}
};

composed_hmodel::composed_hmodel(IloEnv& env, const problem& prob, const std::string& code) :
	hybrid_model(env, prob), code(code), pre_spgm(false)
{
	stringstream ss(code);
	string token;
	while (getline(ss, token, '-')) {
		form_codes.push_back(token);
		if (getline(ss, token, '-')) {
			int break_point = std::stoi(token);

			if (break_point <= 1)
				throw invalid_argument("break points must be at least 2");

			// Check if this point is greater than the last point
			if (break_points.size() && break_point <= break_points.back())
				throw invalid_argument("break points must be in increasing order");

			break_points.push_back(break_point);
		}
		else
			break;
	}
	if (form_codes.size() == break_points.size())
		form_codes.push_back("ustd");	// Default fallback

	assert(form_codes.size() == break_points.size() + 1);

	// Check if the codes are valid
	for (int i = 0; i < form_codes.size() - 1; i++)
		if (!VALID_CODES.count(form_codes[i]))
			throw invalid_argument("invalid code: " + form_codes[i]);

	if (!VALID_FALLBACK.count(form_codes.back()))
		throw invalid_argument("invalid fallback code: " + form_codes.back());
}

vector<formulation*> composed_hmodel::assign_formulations()
{
	int filtered_count = 0;
	vector<int> counts(form_codes.size(), 0);
	vector<string> form_names(form_codes.size());

	// Formulation names
	std::transform(form_codes.begin(), form_codes.end() - 1, form_names.begin(),
				   [&](const string& code) {
					   return VALID_CODES.at(code);
				   });
	form_names.back() = VALID_FALLBACK.at(form_codes.back());

	std::vector<formulation*> all_forms(K);
	spgm_preprocessor spgm_preproc;

	LOOP(k, K) {
		light_graph* graph;

		if (pre_spgm) {
			auto info = spgm_preproc.preprocess(prob, k);
			graph = new light_graph(info.build_graph());
		}
		else {
			graph = new light_graph(prob.graph);
		}
		auto paths = graph->bilevel_feasible_paths_2(prob.commodities[k].origin,
													 prob.commodities[k].destination,
													 break_points.empty() ? 2 : (break_points.back() + 1));

		if (paths.size() <= 1) {
			all_forms[k] = new null_formulation();
			filtered_count++;
		}
		else {
			auto it = std::find_if(break_points.begin(), break_points.end(),
								   [&](int point) {
									   return paths.size() <= point;
								   });

			// Fallback
			if (it == break_points.end()) {
				string& form_code = form_codes.back();
				if (form_code == "ustd")
					all_forms[k] = new standard_formulation();
				else if(form_code == "spgm")
					all_forms[k] = new standard_formulation(new spgm_preprocessor());
				else
					throw runtime_error("fallback code " + form_code + " is not supported");
				counts.back()++;
			}
			// General
			else {
				int i = std::distance(break_points.begin(), it);
				string& form_code = form_codes[i];
				if (form_code == "spgm")
					all_forms[k] = new standard_formulation(new spgm_preprocessor());
				else if (form_code == "sstd")
					all_forms[k] = new sstd_formulation(paths, *graph);
				else
					all_forms[k] = new general_formulation(paths, *graph, form_code);
				counts[i]++;
			}
		}

		delete graph;
	}

	cout << "CODE: " << code << endl;
	cout << "Filtered: " << filtered_count << endl;
	for (int i = 0; i < form_codes.size(); i++) {
		cout << form_names[i] << ": " << counts[i] << endl;
	}

	return all_forms;
}

void composed_hmodel::config(const model_config& conf)
{
	hybrid_model::config(conf);
	pre_spgm = conf.pre_spgm;
}