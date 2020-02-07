#include "benders_model_original.h"

#include "../macros.h"
#include "model_utils.h"
#include <iostream>
#include <sstream>
#include <chrono>

using namespace std;
using namespace boost;

ILOLAZYCONSTRAINTCALLBACK1(benders_model_original_callback, benders_model_original&, bmodel) {
	using NumArray = benders_model_original::NumArray;
	using NumMatrix = benders_model_original::NumMatrix;

	IloEnv env = getEnv();

	// Extract x values
	NumMatrix xvals(env, bmodel.K);
	LOOP(k, bmodel.K) {
		xvals[k] = NumArray(env, bmodel.A1);
		getValues(xvals[k], bmodel.x[k]);
	}

	// Separation algorithm
	IloExpr cut_lhs(env);
	IloNum cut_rhs = 0;
	bmodel.separate(xvals, cut_lhs, cut_rhs);

	// Add the cut
	add(cut_lhs >= cut_rhs).end();

	// Clean up
	cut_lhs.end();
	clean_up(xvals);
};

benders_model_original::benders_model_original(IloEnv& env, const problem& _prob) :
	model_with_callback(env), model_single(_prob), submodel(env), subcplex(env), const_val_map(),
	separate_time(0), subprob_time(0), separate_count(0) {

	// Variables
	v = IloNumVar(env, 0, prob.get_obj_upper_bound(), "v");
	x = NumVarMatrix(env, K);

	LOOP(k, K) {
		x[k] = NumVarArray(env, A1, 0, 1, ILOBOOL);
		LOOP(a, A1) {
			SRC_DST_FROM_A1(prob, a);

			char name[20];
			sprintf(name, "x[%d,%d->%d]", k, src, dst);
			x[k][a].setName(name);
		}
	}

	// Objective
	obj = IloMaximize(env, v);

	// Add to model
	cplex_model.add(obj);
	LOOP(k, K) cplex_model.add(x[k]);

	// Init subproblem
	init_subproblem();

	// Dual values
	mu = NumMatrix(env, K);
	f = NumMatrix(env, K);
	rho = NumArray(env, K);
	alpha = NumMatrix(env, K);
	beta = NumMatrix(env, K);
	LOOP(k, K) {
		mu[k] = NumArray(env, V);
		f[k] = NumArray(env, A);
		alpha[k] = NumArray(env, A1);
		beta[k] = NumArray(env, A1);
	}
}

void benders_model_original::init_subproblem()
{
	// Variables and naming
	y = NumVarMatrix(env, K);
	lambda = NumVarMatrix(env, K);
	tx = NumVarMatrix(env, K);
	t = NumVarArray(env, A1, 0, IloInfinity);

	LOOP(k, K) {
		y[k] = NumVarArray(env, A2, 0, IloInfinity);
		lambda[k] = NumVarArray(env, V, -IloInfinity, IloInfinity);
		tx[k] = NumVarArray(env, A1, 0, IloInfinity);

		LOOP(a, A2) {
			SRC_DST_FROM_A2(prob, a);
			char name[20];
			sprintf(name, "y[%d,%d->%d]", k, src, dst);
			y[k][a].setName(name);
		}

		LOOP(i, V) {
			char name[20];
			sprintf(name, "lambda[%d,%d]", k, i);
			lambda[k][i].setName(name);
		}

		LOOP(a, A1) {
			SRC_DST_FROM_A1(prob, a);
			char name[20];
			sprintf(name, "tx[%d,%d->%d]", k, src, dst);
			tx[k][a].setName(name);
		}
	}

	LOOP(a, A1) {
		SRC_DST_FROM_A1(prob, a);
		char name[20];
		sprintf(name, "t[%d->%d]", src, dst);
		t[a].setName(name);
	}

	// Objective
	subobj = IloMaximize(env);
	LOOP(k, K) LOOP(a, A1) subobj.setLinearCoef(tx[k][a], prob.commodities[k].demand);

	// Flow constraints
	flow_constr = RangeMatrix(env, K);
	LOOP(k, K) {
		flow_constr[k] = RangeArray(env, V, 0, 0);

		// Supply and demand (depends on x)
		//flow_constr[k][prob.commodities[k].origin].setBounds(1, 1);
		//flow_constr[k][prob.commodities[k].destination].setBounds(-1, -1);
	}

	// Flow matrix (y-only)
	LOOP(a, A2) {
		SRC_DST_FROM_A2(prob, a);

		LOOP(k, K) flow_constr[k][src].setLinearCoef(y[k][a], 1);
		LOOP(k, K) flow_constr[k][dst].setLinearCoef(y[k][a], -1);
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

	// Equal objective (y-only)
	equal_obj = RangeArray(env, K, 0, 0);	// Bounds depend on x
	LOOP(a, A2) {
		auto edge = A2_TO_EDGE(prob, a);
		cost_type cost = prob.cost_map[edge];
		LOOP(k, K) equal_obj[k].setLinearCoef(y[k][a], cost);
	}
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
		submodel.add(flow_constr[k]);
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

void benders_model_original::update_subproblem(const NumMatrix& xvals)
{
	// Flow matrix
	LOOP(k, K) {
		vector<int> supply(V, 0);
		supply[prob.commodities[k].origin] = 1;
		supply[prob.commodities[k].destination] = -1;

		LOOP(a, A1) {
			if (xvals[k][a] < 0.5) continue;		// Not a chosen edge

			SRC_DST_FROM_A1(prob, a);

			supply[src] -= 1;
			supply[dst] += 1;
		}

		LOOP(i, V) flow_constr[k][i].setBounds(supply[i], supply[i]);
	}

	// Equal objective
	LOOP(k, K) {
		cost_type rhs = 0;

		LOOP(a, A1) {
			if (xvals[k][a] < 0.5) continue;		// Not a chosen edge

			auto edge = A1_TO_EDGE(prob, a);
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

void benders_model_original::update_const_val_map() {
	LOOP(k, K) {
		LOOP(i, V) const_val_map[flow_constr[k][i].getId()] = &mu[k][i];
		LOOP(a, A) const_val_map[dual_feas[k][a].getId()] = &f[k][a];
		const_val_map[equal_obj[k].getId()] = &rho[k];
		LOOP(a, A1) const_val_map[bilinear1[k][a].getId()] = &alpha[k][a];
		LOOP(a, A1) const_val_map[bilinear2[k][a].getId()] = &beta[k][a];
	}
}

void benders_model_original::separate(const NumMatrix& xvals, IloExpr& cut_lhs, IloNum& cut_rhs)
{
	auto start = chrono::high_resolution_clock::now();

	// Update and resolve subproblem
	update_subproblem(xvals);

	auto substart = chrono::high_resolution_clock::now();
	bool is_feasible = subcplex.solve();
	auto subend = chrono::high_resolution_clock::now();

	// Extract dual information
	if (is_feasible) {
		// Primal feasible, get dual values
		LOOP(k, K) {
			subcplex.getDuals(mu[k], flow_constr[k]);
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
		NumArray y(env, 0);

		subcplex.dualFarkas(constArray, y);

		for (int i = 0; i < constArray.getSize(); i++) {
			IloNum* val_ref = const_val_map[constArray[i].getId()];
			if (val_ref)
				(*val_ref) = -y[i];
		}

		constArray.end();
		y.end();
	}

	// Cut formulation
	LOOP(k, K) {
		// Right-hand side
		cut_rhs -= mu[k][prob.commodities[k].origin];
		cut_rhs += mu[k][prob.commodities[k].destination];
		LOOP(a, A) {
			auto edge = A_TO_EDGE(prob, a);
			cut_rhs -= prob.cost_map[edge] * f[k][a];
		}
		LOOP(a, A1) {
			cut_rhs -= prob.big_n[a] * beta[k][a];
		}

		// Left-hand side
		LOOP(a, A1) {
			SRC_DST_FROM_A1(prob, a);
			cost_type cost = prob.cost_map[edge];

			cut_lhs.setLinearCoef(x[k][a],
								  -mu[k][src] + mu[k][dst] - cost * rho[k] + prob.big_m[k][a] * alpha[k][a] - prob.big_n[a] * beta[k][a]);
		}
	}

	// Optimality cut
	if (is_feasible) {
		cut_lhs.setLinearCoef(v, -1);
	}

	// Utilities
	auto end = chrono::high_resolution_clock::now();
	separate_time += chrono::duration<double>(end - start).count();
	subprob_time += chrono::duration<double>(subend - substart).count();
	++separate_count;
}

solution benders_model_original::get_solution()
{
	// Resolve subproblem to get y and t (must be feasible)
	NumMatrix xvals = get_values(cplex, x);

	update_subproblem(xvals);
	subcplex.solve();

	NumMatrix yvals = get_values(subcplex, y);
	NumArray tvals = get_values(subcplex, t);

	solution sol = fetch_solution_from_xy_t(*this, xvals, yvals, tvals);

	clean_up(xvals);
	clean_up(yvals);
	tvals.end();

	return sol;
}

std::string benders_model_original::get_report()
{
	ostringstream ss;
	ss << "OBJ: " << cplex.getObjValue() << endl <<
		"TIME: " << get_time() << " s" <<
		"    Sep " << separate_time << " s" <<
		"    Avg " << (separate_time * 1000 / separate_count) << " ms" <<
		"    Sub " << (subprob_time * 100 / separate_time) << "%" << endl <<
		"SEP: Total " << separate_count << endl;

	return ss.str();
}

IloCplex::Callback benders_model_original::attach_callback()
{
	return cplex.use(benders_model_original_callback(env, *this));
}
