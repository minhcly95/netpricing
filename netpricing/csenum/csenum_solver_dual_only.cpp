#include "csenum_solver_dual_only.h"
#include "../macros.h"

csenum_solver_dual_only::csenum_solver_dual_only(const IloEnv& _env, const problem& _prob) :
	csenum_solver_base(_env, _prob), dual_model(env), dual_cplex(dual_model)
{
	build_dual_model();
}

void csenum_solver_dual_only::build_dual_model()
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

bool csenum_solver_dual_only::solve_dual()
{
	return dual_cplex.solve();
}

double csenum_solver_dual_only::get_dual_cost()
{
	return dual_cplex.getObjValue();
}

cplex_def::NumMatrix csenum_solver_dual_only::get_lambda_impl(NumMatrix& lvals)
{
	LOOP(k, K) dual_cplex.getValues(lambda[k], lvals[k]);
	return lvals;
}

cplex_def::NumArray csenum_solver_dual_only::get_t_impl(NumArray& tvals)
{
	dual_cplex.getValues(t, tvals);
	return tvals;
}

void csenum_solver_dual_only::clear_dual_state_impl()
{
	// Clear all lower bounds in dual model
	LOOP(k, K) LOOP(a, A) {
		dual_feas[k][a].setLB(-IloInfinity);
	}
}

void csenum_solver_dual_only::push_dual_state_impl(const csenum_coor& coor)
{
	dual_feas[coor.k][coor.a].setLB(dual_feas[coor.k][coor.a].getUB());
}

void csenum_solver_dual_only::pop_dual_state_impl(const csenum_coor& coor)
{
	dual_feas[coor.k][coor.a].setLB(-IloInfinity);
}
