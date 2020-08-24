#include "path_preprocessor.h"

#include "../../problem.h"

#include <iostream>

using namespace std;

path_preprocessor::path_preprocessor(std::vector<path> paths) :
	paths(paths), P(paths.size())
{
}

preprocess_info path_preprocessor::preprocess_impl(const light_graph& graph, const commodity& comm, int k)
{
	preprocess_info info = preprocessor::preprocess_impl(graph, comm, k);

	using arc = pair<int, int>;
	set<arc> keep_A;
	set<int> keep_V;

	for (path& path : paths) {
		keep_V.insert(path[0]);
		for (int i = 0; i < path.size() - 1; i++) {
			keep_A.emplace(path[i], path[i + 1]);
			keep_V.insert(path[i + 1]);
		}
	}

	info.V = std::move(keep_V);

	for (auto it = info.A.begin(); it != info.A.end();) {
		if (!keep_A.count(info.bimap_A.left.at(*it)))
			it = info.A.erase(it);
		else
			it++;
	}

	for (auto it = info.A1.begin(); it != info.A1.end();) {
		if (!keep_A.count(info.bimap_A1.left.at(*it)))
			it = info.A1.erase(it);
		else
			it++;
	}

	for (auto it = info.A2.begin(); it != info.A2.end();) {
		if (!keep_A.count(info.bimap_A2.left.at(*it)))
			it = info.A2.erase(it);
		else
			it++;
	}

	info.reduce(comm.origin, comm.destination);
	info.clean();

	cout << "Comm " << k << ": " << P << " paths, " << info.V.size() << " nodes, "
		<< info.A.size() << " arcs (" << info.A1.size() << " tolled)" << endl;

	return info;
}
