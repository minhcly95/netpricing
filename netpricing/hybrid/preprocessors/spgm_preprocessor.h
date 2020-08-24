#pragma once

#include "../base/preprocessor.h"

struct spgm_preprocessor : public virtual preprocessor {

	virtual preprocess_info preprocess_impl(const light_graph& graph, const commodity& comm, int k = -1) override;
};