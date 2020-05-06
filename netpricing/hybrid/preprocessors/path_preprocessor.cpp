#include "path_preprocessor.h"

#include <iostream>

#include "../../macros.h"

using namespace std;

path_preprocessor::path_preprocessor(std::vector<path> paths) :
	paths(paths), P(paths.size())
{
}

preprocess_info path_preprocessor::preprocess(const problem& prob)
{
	preprocess_info info = preprocessor::preprocess(prob);

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

	int removed_V = info.V.size() - keep_V.size();
	int removed_A = info.A.size() - keep_A.size();

	info.V.erase(
		remove_if(info.V.begin(), info.V.end(),
				  [&](int i) {
					  return keep_V.find(i) == keep_V.end();
				  }),
		info.V.end());

	for (auto it = info.A.begin(); it != info.A.end();) {
		int a = *it;
		arc arc = info.bimap_A.left.at(a);

		if (keep_A.find(arc) == keep_A.end()) {
			// Remove
			bool is_tolled = info.is_tolled.at(a);
			if (is_tolled) {
				int a1 = info.a_to_a1(a);
				info.A1.erase(remove(info.A1.begin(), info.A1.end(), a1), info.A1.end());
				info.bimap_A1.left.erase(a1);
				info.cost_A1.erase(a1);
			}
			else {
				int a2 = info.a_to_a2(a);
				info.A2.erase(remove(info.A2.begin(), info.A2.end(), a2), info.A2.end());
				info.bimap_A2.left.erase(a2);
				info.cost_A2.erase(a2);
			}

			info.bimap_A.left.erase(a);
			info.cost_A.erase(a);
			info.is_tolled.erase(a);

			it = info.A.erase(it);
		}
		else
			it++;
	}

	cout << "No paths " << P << ", eliminated " << removed_V << " nodes, " << removed_A << " arcs" << endl;

	return info;
}
