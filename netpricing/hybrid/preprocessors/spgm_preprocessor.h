#pragma once

#include "../base/preprocessor.h"

struct spgm_preprocessor : preprocessor {
	using path = std::vector<int>;

	virtual preprocess_info preprocess(const problem& prob, int k) override;
};