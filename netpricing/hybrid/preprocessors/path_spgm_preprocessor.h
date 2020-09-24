#pragma once

#include "path_preprocessor.h"
#include "spgm_preprocessor.h"

struct path_spgm_preprocessor : public path_preprocessor, private spgm_preprocessor {
	path_spgm_preprocessor(std::vector<path> paths);

	virtual preprocess_info preprocess_impl(const light_graph& graph, const commodity& comm, int k = -1) override;
};