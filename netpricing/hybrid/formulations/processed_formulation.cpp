#include "processed_formulation.h"

#include "../../macros.h"

using namespace std;

processed_formulation::processed_formulation() :
	paths(), P(0),
	removed_V(), removed_A(), removed_A1(), removed_A2()
{
}

processed_formulation::processed_formulation(const std::vector<path>& paths) :
	paths(paths), P(paths.size()),
	removed_V(), removed_A(), removed_A1(), removed_A2()
{
}

processed_formulation::~processed_formulation()
{
}

void processed_formulation::process_graph()
{
	if (P == 0)
		return;		// Don't process the graph

	using arc = pair<int, int>;
	set<arc> keep_arcs;

	LOOP(i, V) removed_V.insert(i);

	LOOP(p, P) {
		path& path = paths[p];

		removed_V.erase(path[0]);
		for (int i = 0; i < path.size() - 1; i++) {
			keep_arcs.emplace(path[i], path[i + 1]);
			removed_V.erase(path[i + 1]);
		}
	}

	LOOP(a, A1) {
		SRC_DST_FROM_A1(*prob, a);
		if (keep_arcs.find(make_pair(src, dst)) == keep_arcs.end()) {
			removed_A1.insert(a);
			removed_A.insert(A1_TO_A(*prob, a));
		}
	}

	LOOP(a, A2) {
		SRC_DST_FROM_A2(*prob, a);
		if (keep_arcs.find(make_pair(src, dst)) == keep_arcs.end()) {
			removed_A2.insert(a);
			removed_A.insert(A2_TO_A(*prob, a));
		}
	}

	cout << "Commodity " << k <<
		": eliminated " << removed_V.size() << " nodes, " << removed_A.size() << " arcs" << endl;
}
