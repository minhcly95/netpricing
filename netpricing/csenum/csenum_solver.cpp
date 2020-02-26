#include "csenum_solver.h"
#include "../macros.h"

csenum_solver::csenum_solver(const IloEnv& _env, const problem& _prob) :
	model_single(_prob), primal_lgraph(prob.graph),
	env(_env), dual_model(env), dual_cplex(dual_model)
{
	build_dual_model();
}

void csenum_solver::build_dual_model()
{
	lambda = VarMatrix(env, K);
	LOOP(k, K) lambda[k] = VarArray(env, V, -IloInfinity, IloInfinity);

	t = VarArray(env, A1, 0, IloInfinity);

	dual_obj = IloMaximize(env);
	LOOP(k, K) {
		commodity& comm = prob.commodities[k];
		dual_obj.setLinearCoef(lambda[k][comm.origin], comm.demand);
		dual_obj.setLinearCoef(lambda[k][comm.destination], -comm.demand);
	}

	dual_feas = RangeMatrix(env, K);
	LOOP(k, K) dual_feas[k] = RangeArray(env, A);
	LOOP(a, A) {
		SRC_DST_FROM_A(prob, a);
		bool is_tolled = prob.is_tolled_map[edge];
		cost_type cost = prob.cost_map[edge];

		LOOP(k, K) {
			dual_feas[k][a] = IloRange(lambda[k][src] - lambda[k][dst] <= cost);
			if (is_tolled)
				dual_feas[k][a].setLinearCoef(t[A_TO_A1(prob, a)], -1);
		}
	}

	dual_model.add(dual_obj);
	LOOP(k, K) dual_model.add(dual_feas[k]);

	dual_cplex.setOut(env.getNullStream());
}

bool csenum_solver::solve_primal(int k)
{
	commodity& comm = prob.commodities[k];
	primal_result = primal_lgraph.shortest_path(comm.origin, comm.destination);
	return !primal_result.empty();
}

double csenum_solver::get_primal_cost()
{
	return primal_lgraph.get_path_cost(primal_result);
}

std::vector<int> csenum_solver::get_path()
{
	return primal_result;
}

bool csenum_solver::solve_dual()
{
	return dual_cplex.solve();
}

double csenum_solver::get_dual_cost()
{
	return dual_cplex.getObjValue();
}

csenum_solver::NumMatrix csenum_solver::get_lambda()
{
	NumMatrix lvals(env, K);
	LOOP(k, K) {
		lvals[k] = NumArray(env, V);
		dual_cplex.getValues(lambda[k], lvals[k]);
	}
	return lvals;
}

csenum_solver::NumArray csenum_solver::get_t()
{
	NumArray tvals(env, A1);
	dual_cplex.getValues(t, tvals);
	return tvals;
}

csenum_solver::NumMatrix csenum_solver::get_lambda(NumMatrix& lvals)
{
	LOOP(k, K) dual_cplex.getValues(lambda[k], lvals[k]);
	return lvals;
}

csenum_solver::NumArray csenum_solver::get_t(NumArray& tvals)
{
	dual_cplex.getValues(t, tvals);
	return tvals;
}

void csenum_solver::clear_primal_state()
{
	// Enable all edges in primal graph
	LOOP(i, V) {
		for (auto& pair : primal_lgraph.E[i]) {
			pair.second.enabled = true;
		}
	}
	primal_state_stack.clear();
}

void csenum_solver::clear_dual_state()
{
	// Clear all lower bounds in dual model
	LOOP(k, K) LOOP(a, A) {
		dual_feas[k][a].setLB(-IloInfinity);
	}
	dual_state_stack.clear();
}

void csenum_solver::set_primal_state(std::vector<int> as)
{
	clear_primal_state();

	// Disable all edges listed in coork
	for (int a : as)
		push_primal_state(a);
}

void csenum_solver::set_dual_state(std::vector<csenum_coor> coors)
{
	clear_dual_state();

	// Restrict dual_feas[k][a] to equality
	for (auto& coor : coors)
		push_dual_state(coor);
}

void csenum_solver::push_primal_state(int a)
{
	SRC_DST_FROM_A(prob, a);
	primal_lgraph.E[src][dst].enabled = false;
	primal_state_stack.push_back(a);
}

void csenum_solver::push_dual_state(const csenum_coor& coor)
{
	dual_feas[coor.k][coor.a].setLB(dual_feas[coor.k][coor.a].getUB());
	dual_state_stack.push_back(coor);
}

void csenum_solver::pop_primal_state()
{
	int a = primal_state_stack.back();

	SRC_DST_FROM_A(prob, a);
	primal_lgraph.E[src][dst].enabled = true;

	primal_state_stack.pop_back();
}

void csenum_solver::pop_dual_state()
{
	csenum_coor& coor = dual_state_stack.back();

	dual_feas[coor.k][coor.a].setLB(-IloInfinity);

	dual_state_stack.pop_back();
}
