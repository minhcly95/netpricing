#include "compslack_model.h"

#include "../macros.h"
#include "../utilities/set_var_name.h"
#include "model_utils.h"
#include "../graph/light_graph.h"

#include <map>
#include <utility>
#include <sstream>

compslack_model::compslack_model(IloEnv& env, const problem& _prob) : model_cplex(env), model_single(_prob) {
	// Typedef
	using namespace std;
	using namespace boost;
	using graph_type = problem::graph_type;

	// Variables
	x = NumVarMatrix(env, K);
	y = NumVarMatrix(env, K);
	z = NumVarMatrix(env, K);
	lambda = NumVarMatrix(env, K);
	t = NumVarArray(env, A1, 0, IloInfinity);

	LOOP(k, K) {
		x[k] = NumVarArray(env, A1, 0, 1, ILOBOOL);
		y[k] = NumVarArray(env, A2, 0, 1, ILOBOOL);
		z[k] = NumVarArray(env, A);
		lambda[k] = NumVarArray(env, V, -IloInfinity, IloInfinity);
	}

	SET_VAR_NAMES(*this, x, y, t, lambda);

	cplex_model.add(t);
	LOOP(k, K) {
		cplex_model.add(x[k]);
		cplex_model.add(y[k]);
		cplex_model.add(lambda[k]);
	}

	// z is used to reference x and y
	LOOP(a1, A1) {
		int a = A1_TO_A(prob, a1);
		LOOP(k, K) z[k][a] = x[k][a1];
	}
	LOOP(a2, A2) {
		int a = A2_TO_A(prob, a2);
		LOOP(k, K) z[k][a] = y[k][a2];
	}

	// Objective
	obj = IloMaximize(env);
	LOOP(k, K) {
		commodity& comm = prob.commodities[k];
		demand_type demand = comm.demand;

		obj.setLinearCoef(lambda[k][comm.origin], demand);
		obj.setLinearCoef(lambda[k][comm.destination], -demand);

		LOOP(a, A) {
			auto edge = A_TO_EDGE(prob, a);
			cost_type cost = prob.cost_map[edge];
			obj.setLinearCoef(z[k][a], -cost * demand);
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

	// Dual feasibility
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

	// Complementary slackness
	comp_slack = RangeMatrix(env, K);
	LOOP(k, K) comp_slack[k] = RangeArray(env, A);
	light_graph lgraph(prob.graph);
	LOOP(k, K) {
		// Big-M calculation
		lgraph.set_toll_arcs_enabled(true);
		vector<cost_type> lambda_lb = lgraph.price_to_dst(prob.commodities[k].destination);

		lgraph.set_toll_arcs_enabled(false);
		vector<cost_type> lambda_ub = lgraph.price_to_dst(prob.commodities[k].destination);

		LOOP(a, A) {
			SRC_DST_FROM_A(prob, a);
			bool is_tolled = prob.is_tolled_map[edge];
			cost_type cost = prob.cost_map[edge];
			cost_type big_m = cost - lambda_lb[src] + lambda_ub[dst];
			if (is_tolled)
				big_m += prob.big_n[A_TO_A1(prob, a)];

			if (isnan(big_m) || big_m > 10000)
				big_m = 10000;
			if (big_m < 0)
				big_m = 0;

			comp_slack[k][a] = IloRange(-lambda[k][src] + lambda[k][dst] + big_m * z[k][a] <= big_m - cost + 1e-5);
			if (is_tolled)
				comp_slack[k][a].setLinearCoef(t[A_TO_A1(prob, a)], 1);
		}
	}

	// Add to model
	cplex_model.add(obj);
	LOOP(k, K) {
		cplex_model.add(flow_constr[k]);
		cplex_model.add(dual_feas[k]);
		cplex_model.add(comp_slack[k]);
	}
}

solution compslack_model::get_solution()
{
	NumMatrix zvals = get_values(cplex, z);
	NumArray tvals = get_values(cplex, t);

	solution sol = fetch_solution_from_z_t(*this, zvals, tvals);

	clean_up(zvals);
	tvals.end();

	return sol;
}
