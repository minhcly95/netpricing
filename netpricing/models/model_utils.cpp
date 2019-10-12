#include "model_utils.h"

#include <map>

#include "../macros.h"

using namespace std;

solution fetch_solution_from_z_t(const model& m, model::NumMatrix& zvals, model::NumArray& tvals)
{
	solution sol;

	// Paths
	LOOP(k, m.K) {
		// Build the src dst map
		map<int, int> src_dst_map;
		LOOP(a, m.A) {
			SRC_DST_FROM_A(m.prob, a);
			if (zvals[k][a] > 0.5)
				src_dst_map[src] = dst;
		}

		// Build the path
		solution::path path;

		int current = m.prob.commodities[k].origin;
		path.push_back(current);
		while (current != m.prob.commodities[k].destination) {
			current = src_dst_map[current];
			path.push_back(current);
		}

		sol.paths.push_back(std::move(path));
	}

	// Tolls
	LOOP(a, m.A1) {
		sol.tolls.push_back(tvals[a]);
	}

	return sol;
}
