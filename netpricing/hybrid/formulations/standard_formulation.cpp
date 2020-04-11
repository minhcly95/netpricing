#include "standard_formulation.h"

#include "../base/hybrid_model.h"
#include "../../macros.h"
#include "../../utilities/set_var_name.h"

void standard_formulation::formulate_impl()
{
	// Variables
	x = VarArray(env, A1, 0, 1, ILOBOOL);
	y = VarArray(env, A2, 0, IloInfinity);
	lambda = VarArray(env, V, -IloInfinity, IloInfinity);
	tx = VarArray(env, A1, 0, IloInfinity);

	SET_VAR_NAMES_K(model, k, x, y, tx, lambda);

	cplex_model.add(x);
	cplex_model.add(y);
	cplex_model.add(lambda);
	cplex_model.add(tx);

	// Objective
	LOOP(a, A1) obj.setLinearCoef(tx[a], prob->commodities[k].demand);

	// Flow constraints
	flow_constr = RangeArray(env, V, 0, 0);
	flow_constr[prob->commodities[k].origin].setBounds(1, 1);
	flow_constr[prob->commodities[k].destination].setBounds(-1, -1);

	// Flow matrix
	LOOP(a, A1) {
		SRC_DST_FROM_A1(*prob, a);
		flow_constr[src].setLinearCoef(x[a], 1);
		flow_constr[dst].setLinearCoef(x[a], -1);
	}
	LOOP(a, A2) {
		SRC_DST_FROM_A2(*prob, a);
		flow_constr[src].setLinearCoef(y[a], 1);
		flow_constr[dst].setLinearCoef(y[a], -1);
	}

	// Dual feasibility
	dual_feas = RangeArray(env, A);
	LOOP(a, A) {
		SRC_DST_FROM_A(*prob, a);
		bool is_tolled = prob->is_tolled_map[edge];
		cost_type cost = prob->cost_map[edge];

		LOOP(k, K) {
			dual_feas[a] = IloRange(lambda[src] - lambda[dst] <= cost);
			if (is_tolled)
				dual_feas[a].setLinearCoef(t[A_TO_A1(*prob, a)], -1);
		}
	}

	// Equal objective
	equal_obj = IloRange(env, 0, 0);
	LOOP(a, A1) {
		auto edge = A1_TO_EDGE(*prob, a);
		cost_type cost = prob->cost_map[edge];
		equal_obj.setLinearCoef(x[a], cost);
		equal_obj.setLinearCoef(tx[a], 1);
	}
	LOOP(a, A2) {
		auto edge = A2_TO_EDGE(*prob, a);
		cost_type cost = prob->cost_map[edge];
		equal_obj.setLinearCoef(y[a], cost);
	}
	equal_obj.setLinearCoef(lambda[prob->commodities[k].origin], -1);
	equal_obj.setLinearCoef(lambda[prob->commodities[k].destination], 1);

	// Bilinear 1
	bilinear1 = RangeArray(env, A1, -IloInfinity, 0);
	LOOP(a, A1) {
		bilinear1[a].setLinearCoef(tx[a], 1);
		bilinear1[a].setLinearCoef(x[a], -prob->big_m[k][a]);
	}

	// Bilinear 2
	bilinear2 = RangeArray(env, A1);
	LOOP(a, A1) {
		bilinear2[a] = IloRange(env, -IloInfinity, prob->big_n[a]);
		bilinear2[a].setLinearCoef(t[a], 1);
		bilinear2[a].setLinearCoef(tx[a], -1);
		bilinear2[a].setLinearCoef(x[a], prob->big_n[a]);
	}

	// Bilinear 3
	bilinear3 = RangeArray(env, A1, -IloInfinity, 0);
	LOOP(a, A1) {
		bilinear3[a].setLinearCoef(t[a], -1);
		bilinear3[a].setLinearCoef(tx[a], 1);
	}

	// Add to model
	cplex_model.add(flow_constr);
	cplex_model.add(dual_feas);
	cplex_model.add(equal_obj);
	cplex_model.add(bilinear1);
	cplex_model.add(bilinear2);
	cplex_model.add(bilinear3);
}
