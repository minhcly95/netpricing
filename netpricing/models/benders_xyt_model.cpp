#include "benders_xyt_model.h"

#include "../macros.h"
#include "../utilities/set_var_name.h"
#include "model_utils.h"

#include <iostream>
#include <sstream>
#include <chrono>

using namespace std;
using namespace boost;

ILOLAZYCONSTRAINTCALLBACK1(benders_xyt_model_callback, benders_xyt_model&, bmodel) {
	using NumArray = benders_xyt_model::NumArray;
	using NumMatrix = benders_xyt_model::NumMatrix;
	using RangeArray = benders_xyt_model::RangeArray;

	IloEnv env = getEnv();

	// Extract x values
	NumMatrix xvals(env, bmodel.K);
	LOOP(k, bmodel.K) {
		xvals[k] = NumArray(env, bmodel.A1);
		getValues(xvals[k], bmodel.x[k]);
	}
	
	// Extract y values
	NumMatrix yvals(env, bmodel.K);
	LOOP(k, bmodel.K) {
		yvals[k] = NumArray(env, bmodel.A2);
		getValues(yvals[k], bmodel.y[k]);
	}

	// Extract T values
	NumArray tvals(env, bmodel.A1);
	getValues(tvals, bmodel.t);

	// Separation algorithm
	RangeArray cuts(env);
	bmodel.separate(xvals, yvals, tvals, cuts);

	// Add the cut
	LOOP(i, cuts.getSize())
		add(cuts[i]).end();

	// Clean up
	clean_up(xvals);
	clean_up(yvals);
	tvals.end();
	cuts.end();
};

benders_xyt_model::benders_xyt_model(IloEnv& env, const problem& _prob) :
	model_with_callback(env), model_single(_prob), submodel(env, K), subcplex(env, K), const_val_map(),
	separate_time(0), subprob_time(0), separate_count(0) {

	// Variables
	x = NumVarMatrix(env, K);
	y = NumVarMatrix(env, K);
	tx = NumVarMatrix(env, K);
	t = NumVarArray(env, A1, 0, IloInfinity);

	LOOP(k, K) {
		x[k] = NumVarArray(env, A1, 0, 1, ILOBOOL);
		y[k] = NumVarArray(env, A2, 0, IloInfinity);
		tx[k] = NumVarArray(env, A1, 0, IloInfinity);
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
	LOOP(a, A1) {
		SRC_DST_FROM_A1(prob, a);
		LOOP(k, K) flow_constr[k][src].setLinearCoef(x[k][a], 1);
		LOOP(k, K) flow_constr[k][dst].setLinearCoef(x[k][a], -1);
	}
	LOOP(a, A2) {
		SRC_DST_FROM_A2(prob, a);
		LOOP(k, K) flow_constr[k][src].setLinearCoef(y[k][a], 1);
		LOOP(k, K) flow_constr[k][dst].setLinearCoef(y[k][a], -1);
	}

	// Bilinear 1
	bilinear1 = RangeMatrix(env, K);
	LOOP(k, K) {
		bilinear1[k] = RangeArray(env, A1, -IloInfinity, 0);
		LOOP(a, A1) {
			bilinear1[k][a].setLinearCoef(tx[k][a], 1);
			bilinear1[k][a].setLinearCoef(x[k][a], -prob.big_m[k][a]);
		}
	}

	// Bilinear 2
	bilinear2 = RangeMatrix(env, K);
	LOOP(k, K) {
		bilinear2[k] = RangeArray(env, A1);
		LOOP(a, A1) {
			bilinear2[k][a] = IloRange(env, -IloInfinity, prob.big_n[a]);
			bilinear2[k][a].setLinearCoef(t[a], 1);
			bilinear2[k][a].setLinearCoef(tx[k][a], -1);
			bilinear2[k][a].setLinearCoef(x[k][a], prob.big_n[a]);
		}
	}

	// Bilinear 3
	bilinear3 = RangeMatrix(env, K);
	LOOP(k, K) {
		bilinear3[k] = RangeArray(env, A1, -IloInfinity, 0);
		LOOP(a, A1) {
			bilinear3[k][a].setLinearCoef(t[a], -1);
			bilinear3[k][a].setLinearCoef(tx[k][a], 1);
		}
	}

	// Add to model
	cplex_model.add(obj);
	cplex_model.add(t);
	LOOP(k, K) {
		cplex_model.add(x[k]);
		cplex_model.add(y[k]);
		cplex_model.add(tx[k]);

		cplex_model.add(flow_constr[k]);
		cplex_model.add(bilinear1[k]);
		cplex_model.add(bilinear2[k]);
		cplex_model.add(bilinear3[k]);
	}

	// Add valid ineualities
	add_valid_inequalities();

	// Init subproblem
	init_subproblem();

	// Dual values
	f = NumMatrix(env, K);
	rho = NumArray(env, K);
	LOOP(k, K) {
		f[k] = NumArray(env, A);
	}

	// Variable names
	SET_VAR_NAMES(*this, x, y, t, tx, lambda);
}

void benders_xyt_model::add_valid_inequalities() {
	// Unique in/out
	unique_out = make_range_matrix(env, K, V);
	unique_in = make_range_matrix(env, K, V);

	LOOP(k, K) LOOP(i, V) {
		unique_out[k][i] = IloRange(env, -IloInfinity, 1);
		unique_in[k][i] = IloRange(env, -IloInfinity, 1);
	}

	LOOP(a, A1) {
		SRC_DST_FROM_A1(prob, a);
		LOOP(k, K) {
			unique_out[k][src].setLinearCoef(x[k][a], 1);
			unique_in[k][dst].setLinearCoef(x[k][a], 1);
		}
	}

	LOOP(a, A2) {
		SRC_DST_FROM_A2(prob, a);
		LOOP(k, K) {
			unique_out[k][src].setLinearCoef(y[k][a], 1);
			unique_in[k][dst].setLinearCoef(y[k][a], 1);
		}
	}

	LOOP(k, K) {
		cplex_model.add(unique_out[k]);
		cplex_model.add(unique_in[k]);
	}
}

void benders_xyt_model::init_subproblem()
{
	// Variables 
	lambda = NumVarMatrix(env, K);
	LOOP(k, K) {
		lambda[k] = NumVarArray(env, V, -IloInfinity, IloInfinity);
	}

	// Dual feasibility
	dual_feas = RangeMatrix(env, K);
	LOOP(k, K) dual_feas[k] = RangeArray(env, A);
	LOOP(a, A) {
		SRC_DST_FROM_A(prob, a);
		cost_type cost = prob.cost_map[edge];

		LOOP(k, K) {
			dual_feas[k][a] = IloRange(lambda[k][src] - lambda[k][dst] <= cost);
			// The cost of toll arcs depends on T
		}
	}

	// Equal objective
	equal_obj = RangeArray(env, K, 0, 0);	// Bounds depend on x, y and T
	LOOP(k, K) {
		equal_obj[k].setLinearCoef(lambda[k][prob.commodities[k].origin], -1);
		equal_obj[k].setLinearCoef(lambda[k][prob.commodities[k].destination], 1);
	}
			
	// Add to model
	LOOP(k, K) {
		submodel[k] = IloModel(env);
		submodel[k].add(IloMaximize(env));
		submodel[k].add(dual_feas[k]);
		submodel[k].add(equal_obj[k]);

		subcplex[k] = IloCplex(env);
		subcplex[k].extract(submodel[k]);
		subcplex[k].setParam(IloCplex::PreInd, IloFalse);
		subcplex[k].setParam(IloCplex::RootAlg, IloCplex::Dual);
		subcplex[k].setParam(IloCplex::Param::Simplex::Tolerances::Feasibility, 1e-4);
		subcplex[k].setOut(env.getNullStream());
	}
}

void benders_xyt_model::update_subproblem(const NumMatrix& xvals, const NumMatrix& yvals, const NumArray& tvals)
{
	// Dual feasibility
	LOOP(a1, A1) {
		auto edge = A1_TO_EDGE(prob, a1);
		cost_type tolled_cost = prob.cost_map[edge] + tvals[a1];

		LOOP(k, K) dual_feas[k][A1_TO_A(prob, a1)].setUB(tolled_cost);
	}

	// Equal objective
	LOOP(k, K) {
		cost_type rhs = 0;

		LOOP(a, A1) {
			if (xvals[k][a] < 0.5) continue;		// Not a chosen edge

			auto edge = A1_TO_EDGE(prob, a);
			rhs -= prob.cost_map[edge] + tvals[a];
		}

		LOOP(a, A2) {
			if (yvals[k][a] < 0.5) continue;		// Not a chosen edge

			auto edge = A2_TO_EDGE(prob, a);
			rhs -= prob.cost_map[edge];
		}

		equal_obj[k].setBounds(rhs, rhs);
	}

}

void benders_xyt_model::update_const_val_map(int k) {
	LOOP(a, A) const_val_map[dual_feas[k][a].getId()] = &f[k][a];
	const_val_map[equal_obj[k].getId()] = &rho[k];
}

void benders_xyt_model::separate(const NumMatrix& xvals, const NumMatrix& yvals, const NumArray& tvals, RangeArray& cuts) {
	auto start = chrono::high_resolution_clock::now();

	separate_inner(xvals, yvals, tvals, cuts);

	cout << "SEP " << cuts.getSize() << " cuts" << endl;

	auto end = chrono::high_resolution_clock::now();
	separate_time += chrono::duration<double>(end - start).count();
	++separate_count;
}

void benders_xyt_model::separate_inner(const NumMatrix& xvals, const NumMatrix& yvals, const NumArray& tvals, RangeArray& cuts)
{
	// Update subproblem
	update_subproblem(xvals, yvals, tvals);

	LOOP(k, K) {
		// Resolve subproblem
		auto substart = chrono::high_resolution_clock::now();
		bool is_feasible = subcplex[k].solve();
		auto subend = chrono::high_resolution_clock::now();
		subprob_time += chrono::duration<double>(subend - substart).count();

		// If feasible, nothing to do here
		if (is_feasible) {
			continue;
		}

		// Primal infeasible, get dual Farkas certificate (dual extreme ray)
		update_const_val_map(k);

		IloConstraintArray constArray(env, 0);
		NumArray vals(env, 0);

		subcplex[k].dualFarkas(constArray, vals);

		for (int i = 0; i < constArray.getSize(); i++) {
			IloNum* val_ref = const_val_map[constArray[i].getId()];
			if (val_ref)
				(*val_ref) = -vals[i];
		}

		constArray.end();
		vals.end();

		// Cut formulation
		// Right-hand side
		IloNum cut_rhs = 0;
		LOOP(a, A) {
			auto edge = A_TO_EDGE(prob, a);
			cut_rhs -= prob.cost_map[edge] * f[k][a];
		}

		// Left-hand side
		IloExpr cut_lhs(env);
		LOOP(a, A1) {
			SRC_DST_FROM_A1(prob, a);
			cost_type cost = prob.cost_map[edge];

			cut_lhs.setLinearCoef(x[k][a], - cost * rho[k]);
			cut_lhs.setLinearCoef(t[a], f[k][a]);
			cut_lhs.setLinearCoef(tx[k][a], -rho[k]);
		}
		LOOP(a, A2) {
			SRC_DST_FROM_A2(prob, a);
			cost_type cost = prob.cost_map[edge];

			cut_lhs.setLinearCoef(y[k][a], -cost * rho[k]);
		}

		cuts.add(cut_lhs >= cut_rhs);
	}
}

solution benders_xyt_model::get_solution()
{
	// Resolve subproblem to get y (must be feasible)
	NumMatrix xvals = get_values(cplex, x);
	NumMatrix yvals = get_values(cplex, y);
	NumArray tvals = get_values(cplex, t);

	solution sol = fetch_solution_from_xy_t(*this, xvals, yvals, tvals);

	clean_up(xvals);
	clean_up(yvals);
	tvals.end();

	return sol;
}

std::string benders_xyt_model::get_report()
{
	ostringstream ss;
	ss << model_cplex::get_report();
	ss <<
		"SEP: " << separate_count <<
		"    Time " << separate_time << " s" <<
		"    Avg " << (separate_time * 1000 / separate_count) << " ms" <<
		"    Sub " << (subprob_time * 100 / separate_time) << "%" << endl;

	return ss.str();
}

IloCplex::Callback benders_xyt_model::attach_callback()
{
	return cplex.use(benders_xyt_model_callback(env, *this));
}
