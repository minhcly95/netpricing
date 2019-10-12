#pragma once

#include "problem.h"

#include <vector>
#include <nlohmann/json.hpp>

struct solution {

	using path = std::vector<int>;

	std::vector<path> paths;
	std::vector<cost_type> tolls;

	solution();

	solution(const solution& other);
	solution(solution&& other);

	nlohmann::json get_json(const problem& prob) const;
};
