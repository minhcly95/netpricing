#pragma once

#include "problem.h"
#include "problem_multi.h"

#include <vector>
#include "nlohmann/json.hpp"

struct solution {

	using path = std::vector<int>;

	std::vector<path> paths;
	std::vector<cost_type> tolls;

	solution();

	solution(const solution& other);
	solution(solution&& other);

	cost_type get_obj_value(const problem& prob) const;

	nlohmann::json get_json(const problem& prob) const;
	nlohmann::json get_json(const problem_multi& prob) const;
};
