#pragma once

#include "../base/preprocessor.h"

struct path_preprocessor : public virtual preprocessor {
	using path = std::vector<int>;

	int P;
	std::vector<path> paths;

	path_preprocessor(std::vector<path> paths);

	virtual preprocess_info preprocess_impl(const light_graph& graph, const commodity& comm, int k = -1) override;
};