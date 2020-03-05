#include "csenum_solver_excl.h"
#include "../macros.h"
#include <map>

csenum_solver_excl::csenum_solver_excl(const IloEnv& _env, const problem& _prob) :
	csenum_solver_dual_only(_env, _prob), primal_models(), primal_cplex()
{
	build_primal_model();
}

void csenum_solver_excl::build_primal_model()
{
	LOOP(k, K) {
		primal_models.emplace_back(env);
		primal_cplex.emplace_back(primal_models[k]);
	}

	// Variables
	z = NumVarMatrix(env, K);

	LOOP(k, K) z[k] = NumVarArray(env, A, 0, 1);

	// Objective
	LOOP(k, K) {
		primal_objs.emplace_back(IloMinimize(env));
		LOOP(a, A) {
			auto edge = A_TO_EDGE(prob, a);
			primal_objs[k].setLinearCoef(z[k][a], prob.cost_map[edge]);
		}
	}

	// Flow constraints
	flow_constr = RangeMatrix(env, K);
	LOOP(k, K) {
		flow_constr[k] = RangeArray(env, V, 0, 0);

		// Supply and demand
		flow_constr[k][prob.commodities[k].origin].setBounds(1, 1);
		flow_constr[k][prob.commodities[k].destination].setBounds(-1, -1);
	}

	// Flow matrix
	LOOP(a, A) {
		SRC_DST_FROM_A(prob, a);

		LOOP(k, K) flow_constr[k][src].setLinearCoef(z[k][a], 1);
		LOOP(k, K) flow_constr[k][dst].setLinearCoef(z[k][a], -1);
	}

	// Add to model
	LOOP(k, K) {
		primal_models[k].add(primal_objs[k]);
		primal_models[k].add(flow_constr[k]);
		primal_cplex[k].setOut(env.getNullStream());
	}
}

bool csenum_solver_excl::solve_primal(int k)
{
	return primal_cplex[k].solve();
}

std::vector<int> csenum_solver_excl::get_primal_arcs(int k)
{
	NumArray zvals(env, A);
	primal_cplex[k].getValues(z[k], zvals);

	std::vector<int> arcs;
	LOOP(a, A) {
		if (zvals[a] > 0.5)
			arcs.push_back(a);
	}

	return arcs;
}

double csenum_solver_excl::get_primal_cost(int k)
{
	return primal_cplex[k].getObjValue();
}

void csenum_solver_excl::clear_primal_state_impl()
{
	LOOP(k, K) LOOP(a, A) z[k][a].setUB(1);
}

void csenum_solver_excl::clear_dual_state_impl()
{
	csenum_solver_dual_only::clear_dual_state_impl();
	LOOP(k, K) LOOP(a, A) z[k][a].setLB(0);
}

void csenum_solver_excl::push_primal_state_impl(const csenum_coor& coor)
{
	z[coor.k][coor.a].setUB(0);
}

void csenum_solver_excl::push_dual_state_impl(const csenum_coor& coor)
{
	csenum_solver_dual_only::push_dual_state_impl(coor);
	z[coor.k][coor.a].setLB(1);
}

void csenum_solver_excl::pop_primal_state_impl(const csenum_coor& coor)
{
	z[coor.k][coor.a].setUB(1);
}

void csenum_solver_excl::pop_dual_state_impl(const csenum_coor& coor)
{
	csenum_solver_dual_only::pop_dual_state_impl(coor);
	z[coor.k][coor.a].setLB(0);
}
