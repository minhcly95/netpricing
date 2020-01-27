#include "standard_model_multi.h"

#include "../macros.h"
#include "model_utils.h"

#include <map>
#include <utility>
#include <sstream>

standard_model_multi::standard_model_multi(IloEnv& env, const problem_multi& _prob) : model_base(env), model_multi(_prob) {
	// Typedef
	using namespace std;
	using namespace boost;
	using graph_type = problem::graph_type;

	// Variables
	x = NumVarMatrix(env, K);
	y = NumVarMatrix(env, K);
	z = NumVarMatrix(env, K);
	lambda = NumVarMatrix(env, K);
	tx = NumVarMatrix(env, K);
	t = NumVarArray(env, A1, 0, IloInfinity);

	LOOP(k, K) {
		x[k] = NumVarArray(env, A1, 0, 1, ILOBOOL);
		y[k] = NumVarArray(env, A2[k], 0, IloInfinity);
		z[k] = NumVarArray(env, A1 + A2[k]);
		lambda[k] = NumVarArray(env, V, -IloInfinity, IloInfinity);
		tx[k] = NumVarArray(env, A1, 0, IloInfinity);
	}

	// z is used to reference x and y
	LOOP(k, K) LOOP(a1, A1) {
		if (A1_EXISTS(prob, k, a1)) {
			int a = A1_TO_A_MULTI(prob, k, a1);
			z[k][a] = x[k][a1];
		}
	}
	LOOP(k, K) LOOP(a2, A2[k]) {
		int a = A2_TO_A_MULTI(prob, k, a2);
		z[k][a] = y[k][a2];
	}

	// Objective
	obj = IloMaximize(env);
	LOOP(k, K) LOOP(a, A1) obj.setLinearCoef(tx[k][a], prob.commodities[k].demand);

	// Flow constraints
	flow_constr = RangeMatrix(env, K);
	LOOP(k, K) {
		flow_constr[k] = RangeArray(env, V, 0, 0);

		// Supply and demand
		flow_constr[k][prob.commodities[k].origin].setBounds(1, 1);
		flow_constr[k][prob.commodities[k].destination].setBounds(-1, -1);
	}

	// Flow matrix
	LOOP(k, K) LOOP(a, A) IF_A_EXISTS(prob, k, a) {
		auto edge = prob.alledges_index_maps[k].left.at(a);
		int src = source(edge, prob.graphs[k]);
		int dst = target(edge, prob.graphs[k]);

		flow_constr[k][src].setLinearCoef(z[k][a], 1);
		flow_constr[k][dst].setLinearCoef(z[k][a], -1);
	}

	// Dual feasibility
	dual_feas = RangeMatrix(env, K);
	LOOP(k, K) {
		dual_feas[k] = RangeArray(env, A);
		LOOP(a, A) IF_A_EXISTS(prob, k, a) {
			auto edge = prob.alledges_index_maps[k].left.at(a);
			int src = source(edge, prob.graphs[k]);
			int dst = target(edge, prob.graphs[k]);
			bool is_tolled = prob.is_tolled_maps[k][edge];
			cost_type cost = prob.cost_maps[k][edge];

			dual_feas[k][a] = IloRange(lambda[k][src] - lambda[k][dst] <= cost);
			if (is_tolled)
				dual_feas[k][a].setLinearCoef(t[A_TO_A1_MULTI(prob, k, a)], -1);
		}
	}

	// Equal objective
	equal_obj = RangeArray(env, K, 0, 0);
	LOOP(k, K) LOOP(a, A) {
		if (!A_EXISTS(prob, k, a)) continue;

		auto edge = prob.alledges_index_maps[k].left.at(a);
		cost_type cost = prob.cost_maps[k][edge];
		equal_obj[k].setLinearCoef(z[k][a], cost);
	}
	LOOP(k, K) LOOP(a, A1) IF_A1_EXISTS(prob, k, a)
		equal_obj[k].setLinearCoef(tx[k][a], 1);
	LOOP(k, K) {
		equal_obj[k].setLinearCoef(lambda[k][prob.commodities[k].origin], -1);
		equal_obj[k].setLinearCoef(lambda[k][prob.commodities[k].destination], 1);
	}

	// Bilinear 1
	bilinear1 = RangeMatrix(env, K);
	LOOP(k, K) {
		bilinear1[k] = RangeArray(env, A1);
		LOOP(a, A1) IF_A1_EXISTS(prob, k, a) {
			bilinear1[k][a] = IloRange(env, -IloInfinity, 0);
			bilinear1[k][a].setLinearCoef(tx[k][a], 1);
			bilinear1[k][a].setLinearCoef(x[k][a], -prob.big_m[k][a]);
		}
	}

	// Bilinear 2
	bilinear2 = RangeMatrix(env, K);
	LOOP(k, K) {
		bilinear2[k] = RangeArray(env, A1);
		LOOP(a, A1) IF_A1_EXISTS(prob, k, a) {
			bilinear2[k][a] = IloRange(env, -IloInfinity, prob.big_n[a]);
			bilinear2[k][a].setLinearCoef(t[a], 1);
			bilinear2[k][a].setLinearCoef(tx[k][a], -1);
			bilinear2[k][a].setLinearCoef(x[k][a], prob.big_n[a]);
		}
	}

	// Bilinear 3
	bilinear3 = RangeMatrix(env, K);
	LOOP(k, K) {
		bilinear3[k] = RangeArray(env, A1);
		LOOP(a, A1) IF_A1_EXISTS(prob, k, a) {
			bilinear3[k][a] = IloRange(env, -IloInfinity, 0);
			bilinear3[k][a].setLinearCoef(t[a], -1);
			bilinear3[k][a].setLinearCoef(tx[k][a], 1);
		}
	}

	// Add to model
	cplex_model.add(obj);
	LOOP(k, K) {
		cplex_model.add(flow_constr[k]);
		cplex_model.add(dual_feas[k]);
		cplex_model.add(equal_obj[k]);
		cplex_model.add(bilinear1[k]);
		cplex_model.add(bilinear2[k]);
		cplex_model.add(bilinear3[k]);
	}
}

solution standard_model_multi::get_solution()
{
	NumMatrix zvals = get_values(cplex, z);
	NumArray tvals = get_values(cplex, t);

	solution sol;// = fetch_solution_from_z_t(*this, zvals, tvals);

	clean_up(zvals);
	tvals.end();

	return sol;
}

std::string standard_model_multi::get_report()
{
	using namespace std;

	ostringstream ss;
	ss << "OBJ: " << cplex.getObjValue() << endl <<
		"TIME: " << getTime() << " s" << endl;

	return ss.str();
}
