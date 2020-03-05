#include "csenum_solver_base.h"
#include "../macros.h"

csenum_solver_base::csenum_solver_base(const IloEnv& env, const problem& prob) :
	env(env), model_single(prob)
{
}

std::vector<bool> csenum_solver_base::solve_primals()
{
	std::vector<bool> results(K);
	LOOP(k, K) results[k] = solve_primal(k);
	return results;
}

std::vector<std::vector<int>> csenum_solver_base::get_all_primal_arcs()
{
	std::vector<std::vector<int>> paths;
	LOOP(k, K) paths.push_back(get_primal_arcs(k));
	return paths;
}

std::vector<double> csenum_solver_base::get_primal_costs()
{
	std::vector<double> costs(K);
	LOOP(k, K) costs[k] = get_primal_cost(k);
	return costs;
}

cplex_def::NumMatrix csenum_solver_base::get_lambda(NumMatrix& lvals)
{
	return get_lambda_impl(lvals);
}

cplex_def::NumArray csenum_solver_base::get_t(NumArray& tvals)
{
	return get_t_impl(tvals);
}


cplex_def::NumMatrix csenum_solver_base::get_lambda()
{
	NumMatrix lvals(env, K);
	LOOP(k, K) lvals[k] = NumArray(env, V);
	return get_lambda_impl(lvals);
}

cplex_def::NumArray csenum_solver_base::get_t()
{
	NumArray tvals(env, A1);
	return get_t_impl(tvals);
}

void csenum_solver_base::clear_primal_state()
{
	clear_primal_state_impl();
	primal_state_stack.clear();
}

void csenum_solver_base::clear_dual_state()
{
	clear_dual_state_impl();
	dual_state_stack.clear();
}

void csenum_solver_base::set_primal_state(const std::vector<csenum_coor>& coors)
{
	clear_primal_state();

	// Disable all edges[k][a] listed in coors
	for (auto& coor : coors)
		push_primal_state(coor);
}

void csenum_solver_base::set_dual_state(const std::vector<csenum_coor>& coors)
{
	clear_dual_state();

	// Restrict dual_feas[k][a] to equality
	for (auto& coor : coors)
		push_dual_state(coor);
}

void csenum_solver_base::push_primal_state(const csenum_coor& coor)
{
	push_primal_state_impl(coor);
	primal_state_stack.push_back(coor);
}

void csenum_solver_base::push_dual_state(const csenum_coor& coor)
{
	push_dual_state_impl(coor);
	dual_state_stack.push_back(coor);
}

void csenum_solver_base::pop_primal_state()
{
	csenum_coor& coor = primal_state_stack.back();
	pop_primal_state_impl(coor);
	primal_state_stack.pop_back();
}

void csenum_solver_base::pop_dual_state()
{
	csenum_coor& coor = dual_state_stack.back();
	pop_dual_state_impl(coor);
	dual_state_stack.pop_back();
}
