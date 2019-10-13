#pragma once

#include <boost/program_options.hpp>

struct grid_params {
	int width, height;
};

void validate(boost::any& v, const std::vector<std::string>& values, grid_params* target_type, int);
