#include "solution.h"

#include "macros.h"
#include <utility>

using namespace std;
using json = nlohmann::json;

solution::solution() :
	paths(), tolls() { }

solution::solution(const solution& other) :
	paths(other.paths), tolls(other.tolls) { }

solution::solution(solution&& other) :
	paths(std::move(other.paths)), tolls(std::move(other.tolls)) { }

cost_type solution::get_obj_value(const problem& prob) const
{
	cost_type profit = 0;

	LOOP(k, paths.size()) {
		cost_type profit_k = 0;
		const path& p = paths[k];

		for (int i = 0; i < p.size() - 1; ++i) {
			auto edge = EDGE_FROM_SRC_DST(prob, p[i], p[i + 1]);
			if (prob.is_tolled_map[edge]) {
				profit_k += tolls[EDGE_TO_A1(prob, edge)];
			}
		}

		profit += profit_k * prob.commodities[k].demand;
	}

	return profit;
}

json solution::get_json(const problem& prob) const
{
	json json_obj;

	// Paths
	vector<path> paths_copy = paths;
	LOOP(k, paths_copy.size()) {
		path& path = paths_copy[k];
		LOOP(i, path.size())
			++path[i];
	}
	json_obj["paths"] = json(paths_copy);

	// Tolls
	json tolls_obj;
	LOOP(a, tolls.size()) {
		SRC_DST_FROM_A1(prob, a);
		json arc_obj;
		arc_obj["src"] = src + 1;
		arc_obj["dst"] = dst + 1;
		arc_obj["toll"] = tolls[a];
		tolls_obj.push_back(arc_obj);
	}
	json_obj["tolls"] = tolls_obj;

	return json_obj;
}

json solution::get_json(const problem_multi& prob) const
{
	return json();
}
