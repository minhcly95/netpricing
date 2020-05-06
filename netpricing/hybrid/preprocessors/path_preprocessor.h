#pragma once

#include "../base/preprocessor.h"

struct path_preprocessor : preprocessor {
	using path = std::vector<int>;

	int P;
	std::vector<path> paths;

	path_preprocessor(std::vector<path> paths);

	virtual preprocess_info preprocess(const problem& prob) override;
};