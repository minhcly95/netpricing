#include "path_spgm_preprocessor.h"

using namespace std;

path_spgm_preprocessor::path_spgm_preprocessor(std::vector<path> paths) :
	path_preprocessor(paths)
{
}

preprocess_info path_spgm_preprocessor::preprocess_impl(const light_graph& graph, const commodity& comm, int k)
{
	preprocess_info info = spgm_preprocessor::preprocess_impl(graph, comm, k);
	return path_preprocessor::preprocess_impl(info.build_graph(), comm, k);
}
