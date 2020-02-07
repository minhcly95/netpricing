#include "csenum_node.h"
#include "csenum.h"
#include "../macros.h"

using namespace std;

void csenum_node::update_bound()
{
	bound = dual_obj;
	LOOP(k, primal_objs.size())
		bound -= primal_objs[k];
}

int csenum_node::get_depth() const
{
	return lineage.size();
}

bool csenum_node::is_feasible() const
{
	// No violated means this solution is feasible
	return all_of(violated.begin(), violated.end(), [](const auto& vk) { return vk.empty(); });
}

void csenum_node::update_violated()
{
	violated = vector<vector<int>>(cse->K);

	LOOP(k, cse->K) {
		update_violated(k);
	}
}

void csenum_node::update_violated(int k)
{
	violated[k].clear();

	const vector<int>& path = paths[k];
	for (int i = 0; i < path.size() - 1; ++i) {
		int src = path[i];
		int dst = path[i + 1];
		auto edge = EDGE_FROM_SRC_DST(cse->prob, src, dst);
		cost_type cost = cse->prob.cost_map[edge];
		bool is_tolled = cse->prob.is_tolled_map[edge];

		// Slack calculation
		cost_type slack = cost - lambda[k][src] + lambda[k][dst];
		if (is_tolled)
			slack += t[EDGE_TO_A1(cse->prob, edge)];

		// More slack than tolerance
		if (abs(slack) > TOLERANCE) {
			violated[k].push_back(EDGE_TO_A(cse->prob, edge));
		}
	}
}

int csenum_node::get_num_violated() const
{
	int count = 0;
	LOOP(k, violated.size())
		count += violated[k].size();
	return count;
}

bool csenum_node::operator<(const csenum_node& other) const
{
	return bound < other.bound;
}
