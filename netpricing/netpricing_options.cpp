#include "netpricing_options.h"

#include <string>
#include <iostream>

using namespace std;
namespace po = boost::program_options;

void validate(boost::any& v, const vector<string>& values, grid_params* target_type, int)
{
	po::validators::check_first_occurrence(v);

	if (values.size() != 2)
		throw po::validation_error(po::validation_error::invalid_option_value);
	
	grid_params params;
	params.width = stoi(values[0]);
	params.height = stoi(values[1]);

	v = params;
}
