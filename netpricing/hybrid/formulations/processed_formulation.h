#pragma once

#include "../base/formulation.h"
#include "../base/preprocessor.h"
#include "../../graph/light_graph.h"

struct processed_formulation : public formulation {
	constexpr static double TOLL_PREFERENCE = 0.9999;

	preprocessor* preproc;
	preprocess_info info;

	light_graph* lgraph;

	// Unprocessed version
	processed_formulation();

	// Processed version
	processed_formulation(preprocessor* _preproc);

	virtual ~processed_formulation();

	void preprocess();

	std::vector<int> get_path(const NumArray& tvals);
};
