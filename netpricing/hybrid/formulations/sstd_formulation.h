#pragma once

#include "./general_formulation.h"
#include "../preprocessors/spgm_preprocessor.h"

struct light_graph;

struct sstd_formulation : public general_formulation {
	spgm_preprocessor spgm_preproc;

	sstd_formulation(const std::vector<path>& paths, const light_graph& original);

	virtual void prepare() override;
};
