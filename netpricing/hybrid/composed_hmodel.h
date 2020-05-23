#pragma once

#include "base/hybrid_model.h"
#include "../graph/light_graph.h"

#include <string>
#include <map>

struct composed_hmodel : public hybrid_model {
	light_graph lgraph;
	std::string code;
	std::vector<std::string> form_codes;
	std::vector<int> break_points;

	static std::map<std::string, std::string> VALID_CODES;
	static std::map<std::string, std::string> VALID_FALLBACK;

	composed_hmodel(IloEnv& env, const problem& prob, const std::string& code);

	// Inherited via hybrid_model
	virtual std::vector<formulation*> assign_formulations() override;
};