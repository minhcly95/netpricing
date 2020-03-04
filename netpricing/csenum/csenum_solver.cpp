#include "csenum_solver.h"
#include "../macros.h"

csenum_solver::csenum_solver(const IloEnv& _env, const problem& _prob) :
	csenum_solver_dual_only(_env, _prob), primal_lgraphs(), primal_results(K)
{
	build_primal_model();
}

void csenum_solver::build_primal_model()
{
	LOOP(k, K) primal_lgraphs.emplace_back(prob.graph);
}

bool csenum_solver::solve_primal(int k)
{
	commodity& comm = prob.commodities[k];
	primal_results[k] = primal_lgraphs[k].shortest_path(comm.origin, comm.destination);
	return !primal_results[k].empty();
}

std::vector<int> csenum_solver::get_path(int k)
{
	return primal_results[k];
}

double csenum_solver::get_primal_cost(const std::vector<int>& path)
{
	return primal_lgraphs[0].get_path_cost(path);
}

void csenum_solver::clear_primal_state_impl()
{
	// Enable all edges in primal graph
	LOOP(k, K) LOOP(i, V) {
		for (auto& pair : primal_lgraphs[k].E[i]) {
			pair.second.enabled = true;
		}
	}
}

void csenum_solver::push_primal_state_impl(const csenum_coor& coor)
{
	SRC_DST_FROM_A(prob, coor.a);
	primal_lgraphs[coor.k].E[src][dst].enabled = false;
}

void csenum_solver::pop_primal_state_impl(const csenum_coor& coor)
{
	SRC_DST_FROM_A(prob, coor.a);
	primal_lgraphs[coor.k].E[src][dst].enabled = true;
}
