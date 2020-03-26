#include "benders_xy_model.h"

#include "../macros.h"
#include "../utilities/set_var_name.h"
#include "model_utils.h"

#include <iostream>
#include <sstream>
#include <chrono>

using namespace std;
using namespace boost;

ILOLAZYCONSTRAINTCALLBACK1(benders_xy_model_callback, benders_xy_model&, bmodel) {
	using NumArray = benders_xy_model::NumArray;
	using NumMatrix = benders_xy_model::NumMatrix;
	using RangeArray = benders_xy_model::RangeArray;

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

	// Separation algorithm
	RangeArray cuts(env);
	bmodel.separate(xvals, yvals, cuts);

	// Add the cut
	LOOP(i, cuts.getSize())
		add(cuts[i]).end();

	// Clean up
	clean_up(xvals);
	clean_up(yvals);
};

benders_xy_model::benders_xy_model(IloEnv& env, const problem& _prob) :
	model_with_callback(env), model_single(_prob), submodel(env), subcplex(env), const_val_map(),
	separate_time(0), subprob_time(0), separate_count(0) {

	// Variables
	v = IloNumVar(env, 0, prob.get_obj_upper_bound(), "v");
	x = NumVarMatrix(env, K);
	y = NumVarMatrix(env, K);

	LOOP(k, K) {
		x[k] = NumVarArray(env, A1, 0, 1, ILOBOOL);
		y[k] = NumVarArray(env, A2, 0, IloInfinity);
	}

	// Objective
	obj = IloMaximize(env, v);

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

	// Add to model
	cplex_model.add(v);
	cplex_model.add(obj);
	LOOP(k, K) {
		cplex_model.add(x[k]);
		cplex_model.add(y[k]);
		cplex_model.add(flow_constr[k]);
	}

	// Add valid ineualities
	add_valid_inequalities();

	// Init subproblem
	init_subproblem();

	// Dual values
	f = NumMatrix(env, K);
	rho = NumArray(env, K);
	alpha = NumMatrix(env, K);
	beta = NumMatrix(env, K);
	LOOP(k, K) {
		f[k] = NumArray(env, A);
		alpha[k] = NumArray(env, A1);
		beta[k] = NumArray(env, A1);
	}

	// Variable names
	SET_VAR_NAMES(*this, x, y, t, tx, lambda);
}

void benders_xy_model::add_valid_inequalities() {
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

void benders_xy_model::init_subproblem()
{
	// Variables and naming
	lambda = NumVarMatrix(env, K);
	tx = NumVarMatrix(env, K);
	t = NumVarArray(env, A1, 0, IloInfinity);

	LOOP(k, K) {
		lambda[k] = NumVarArray(env, V, -IloInfinity, IloInfinity);
		tx[k] = NumVarArray(env, A1, 0, IloInfinity);
	}

	// Objective
	subobj = IloMaximize(env);
	LOOP(k, K) LOOP(a, A1) subobj.setLinearCoef(tx[k][a], prob.commodities[k].demand);

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

	// Equal objective
	equal_obj = RangeArray(env, K, 0, 0);	// Bounds depend on x, y
	LOOP(a, A1) LOOP(k, K) equal_obj[k].setLinearCoef(tx[k][a], 1);
	LOOP(k, K) {
		equal_obj[k].setLinearCoef(lambda[k][prob.commodities[k].origin], -1);
		equal_obj[k].setLinearCoef(lambda[k][prob.commodities[k].destination], 1);
	}

	// Bilinear 1
	bilinear1 = RangeMatrix(env, K);
	LOOP(k, K) {
		bilinear1[k] = RangeArray(env, A1, -IloInfinity, 0);	// Bounds depend on x
		LOOP(a, A1) {
			bilinear1[k][a].setLinearCoef(tx[k][a], 1);
			//bilinear1[k][a].setLinearCoef(x[k][a], -prob.big_m[k][a]);
		}
	}

	// Bilinear 2
	bilinear2 = RangeMatrix(env, K);
	LOOP(k, K) {
		bilinear2[k] = RangeArray(env, A1);
		LOOP(a, A1) {
			bilinear2[k][a] = IloRange(env, -IloInfinity, prob.big_n[a]);	// Bounds depend on x
			bilinear2[k][a].setLinearCoef(t[a], 1);
			bilinear2[k][a].setLinearCoef(tx[k][a], -1);
			//bilinear2[k][a].setLinearCoef(x[k][a], prob.big_n[a]);
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
	submodel.add(subobj);
	LOOP(k, K) {
		submodel.add(dual_feas[k]);
		submodel.add(equal_obj[k]);
		submodel.add(bilinear1[k]);
		submodel.add(bilinear2[k]);
		submodel.add(bilinear3[k]);
	}
	subcplex.extract(submodel);
	subcplex.setParam(IloCplex::PreInd, IloFalse);
	subcplex.setParam(IloCplex::RootAlg, IloCplex::Dual);
	subcplex.setParam(IloCplex::Param::Simplex::Tolerances::Feasibility, 1e-4);
	subcplex.setOut(env.getNullStream());
}

void benders_xy_model::update_subproblem(const NumMatrix& xvals, const NumMatrix& yvals)
{
	// Equal objective
	LOOP(k, K) {
		cost_type rhs = 0;

		LOOP(a, A1) {
			if (xvals[k][a] < 0.5) continue;		// Not a chosen edge

			auto edge = A1_TO_EDGE(prob, a);
			rhs -= prob.cost_map[edge];
		}

		LOOP(a, A2) {
			if (yvals[k][a] < 0.5) continue;		// Not a chosen edge

			auto edge = A2_TO_EDGE(prob, a);
			rhs -= prob.cost_map[edge];
		}

		equal_obj[k].setBounds(rhs, rhs);
	}

	// Bilinear
	LOOP(k, K) LOOP(a, A1) {
		// Not a chosen edge
		if (xvals[k][a] < 0.5) {
			bilinear1[k][a].setUB(0);
			bilinear2[k][a].setUB(prob.big_n[a]);
		}
		// Is a chosen edge
		else {
			bilinear1[k][a].setUB(prob.big_m[k][a]);
			bilinear2[k][a].setUB(0);
		}
	}
}

void benders_xy_model::update_const_val_map() {
	LOOP(k, K) {
		LOOP(a, A) const_val_map[dual_feas[k][a].getId()] = &f[k][a];
		const_val_map[equal_obj[k].getId()] = &rho[k];
		LOOP(a, A1) const_val_map[bilinear1[k][a].getId()] = &alpha[k][a];
		LOOP(a, A1) const_val_map[bilinear2[k][a].getId()] = &beta[k][a];
	}
}

void benders_xy_model::separate(const NumMatrix& xvals, const NumMatrix& yvals, RangeArray& cuts) {
	auto start = chrono::high_resolution_clock::now();

	separate_inner(xvals, yvals, cuts);

	auto end = chrono::high_resolution_clock::now();
	separate_time += chrono::duration<double>(end - start).count();
	++separate_count;
}

void benders_xy_model::separate_inner(const NumMatrix& xvals, const NumMatrix& yvals, RangeArray& cuts)
{
	// Update and resolve subproblem
	update_subproblem(xvals, yvals);

	auto substart = chrono::high_resolution_clock::now();
	bool is_feasible = subcplex.solve();
	auto subend = chrono::high_resolution_clock::now();
	subprob_time += chrono::duration<double>(subend - substart).count();

	// Extract dual information
	if (is_feasible) {
		// Primal feasible, get dual values
		LOOP(k, K) {
			subcplex.getDuals(f[k], dual_feas[k]);
			subcplex.getDuals(alpha[k], bilinear1[k]);
			subcplex.getDuals(beta[k], bilinear2[k]);
		}
		subcplex.getDuals(rho, equal_obj);
	}
	else {
		// Primal infeasible, get dual Farkas certificate (dual extreme ray)
		update_const_val_map();

		IloConstraintArray constArray(env, 0);
		NumArray vals(env, 0);

		subcplex.dualFarkas(constArray, vals);

		for (int i = 0; i < constArray.getSize(); i++) {
			IloNum* val_ref = const_val_map[constArray[i].getId()];
			if (val_ref)
				(*val_ref) = -vals[i];
		}

		constArray.end();
		vals.end();
	}

	// Cut formulation
	IloNum cut_rhs = 0;
	IloExpr cut_lhs(env);

	LOOP(k, K) {
		// Right-hand side
		LOOP(a, A) {
			auto edge = A_TO_EDGE(prob, a);
			cut_rhs -= prob.cost_map[edge] * f[k][a];
		}
		LOOP(a, A1) {
			cut_rhs -= prob.big_n[a] * beta[k][a];
		}

		// Left-hand side
		LOOP(a, A1) {
			auto edge = A1_TO_EDGE(prob, a);
			cost_type cost = prob.cost_map[edge];
			cut_lhs.setLinearCoef(x[k][a], -cost * rho[k] + prob.big_m[k][a] * alpha[k][a] - prob.big_n[a] * beta[k][a]);
		}
		LOOP(a, A2) {
			auto edge = A2_TO_EDGE(prob, a);
			cost_type cost = prob.cost_map[edge];
			cut_lhs.setLinearCoef(y[k][a], -cost * rho[k]);
		}
	}

	// Optimality cut
	if (is_feasible) {
		cut_lhs.setLinearCoef(v, -1);
		//cout << "OPT SEP" << endl;
	}
	else {
		//cout << "FEAS SEP" << endl;
	}

	cuts.add(cut_lhs >= cut_rhs);
}

solution benders_xy_model::get_solution()
{
	// Resolve subproblem to get t (must be feasible)
	NumMatrix xvals = get_values(cplex, x);
	NumMatrix yvals = get_values(cplex, y);

	update_subproblem(xvals, yvals);
	subcplex.solve();

	NumArray tvals = get_values(subcplex, t);

	solution sol = fetch_solution_from_xy_t(*this, xvals, yvals, tvals);

	clean_up(xvals);
	clean_up(yvals);
	tvals.end();

	return sol;
}

std::string benders_xy_model::get_report()
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

IloCplex::Callback benders_xy_model::attach_callback()
{
	return cplex.use(benders_xy_model_callback(env, *this));
}
