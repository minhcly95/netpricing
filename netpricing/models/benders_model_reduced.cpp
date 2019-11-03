#include "benders_model_reduced.h"

#include "../macros.h"
#include "model_utils.h"
#include <iostream>
#include <sstream>
#include <chrono>

using namespace std;
using namespace boost;

ILOLAZYCONSTRAINTCALLBACK1(benders_model_reduced_callback, benders_model_reduced&, bmodel) {
	using NumArray = benders_model_reduced::NumArray;
	using NumMatrix = benders_model_reduced::NumMatrix;

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

benders_model_reduced::benders_model_reduced(IloEnv& env, const problem& _prob) :
	model_with_callback(env), model_single(_prob), submodel1(env, K), subcplex1(env, K), submodel3(env), subcplex3(env), const_val_map(),
	separate_time(0), subprob1_time(0), subprob3_time(0), separate_count(0),
	flow_cut_count(0), toll_cut_count(0), opt_cut_count(0) {

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

void benders_model_reduced::init_subproblem()
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

	// SUBPROBLEM 1

	// Objective
	subobj1 = IloArray<IloObjective>(env, K);
	LOOP(k, K) {
		subobj1[k] = IloMinimize(env);
		LOOP(a, A2) subobj1[k].setLinearCoef(y[k][a], prob.cost_map[A2_TO_EDGE(prob, a)]);
	}

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

	// Add to model
	LOOP(k, K) {
		submodel1[k] = IloModel(env);
		submodel1[k].add(subobj1[k]);
		submodel1[k].add(flow_constr[k]);

		subcplex1[k] = IloCplex(env);
		subcplex1[k].extract(submodel1[k]);
		subcplex1[k].setParam(IloCplex::PreInd, IloFalse);
		subcplex1[k].setParam(IloCplex::RootAlg, IloCplex::Dual);
		subcplex1[k].setOut(env.getNullStream());
	}

	// SUBPROBLEM 3

	// Objective
	subobj3 = IloMaximize(env);
	LOOP(k, K) LOOP(a, A1) subobj3.setLinearCoef(tx[k][a], prob.commodities[k].demand);

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

	// Equal objective (xy-excluded)
	equal_obj = RangeArray(env, K, 0, 0);	// Bounds depend on x and y
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
	submodel3.add(subobj3);
	LOOP(k, K) {
		submodel3.add(dual_feas[k]);
		submodel3.add(equal_obj[k]);
		submodel3.add(bilinear1[k]);
		submodel3.add(bilinear2[k]);
		submodel3.add(bilinear3[k]);
	}
	subcplex3.extract(submodel3);
	subcplex3.setParam(IloCplex::PreInd, IloFalse);
	subcplex3.setParam(IloCplex::RootAlg, IloCplex::Dual);
	subcplex3.setParam(IloCplex::Param::Simplex::Tolerances::Feasibility, 1e-4);
	subcplex3.setOut(env.getNullStream());
}

void benders_model_reduced::update_subproblem1(const NumMatrix& xvals)
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
}

void benders_model_reduced::update_subproblem3(const NumMatrix& xvals, const NumMatrix& yvals)
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
			rhs -= prob.cost_map[edge] * yvals[k][a];
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

void benders_model_reduced::update_const_val_map() {
	LOOP(k, K) {
		LOOP(i, V) const_val_map[flow_constr[k][i].getId()] = &mu[k][i];
		LOOP(a, A) const_val_map[dual_feas[k][a].getId()] = &f[k][a];
		const_val_map[equal_obj[k].getId()] = &rho[k];
		LOOP(a, A1) const_val_map[bilinear1[k][a].getId()] = &alpha[k][a];
		LOOP(a, A1) const_val_map[bilinear2[k][a].getId()] = &beta[k][a];
	}
}

void benders_model_reduced::separate(const NumMatrix& xvals, IloExpr& cut_lhs, IloNum& cut_rhs) {
	auto start = chrono::high_resolution_clock::now();

	separate_inner(xvals, cut_lhs, cut_rhs);

	auto end = chrono::high_resolution_clock::now();
	separate_time += chrono::duration<double>(end - start).count();
	++separate_count;
}

void benders_model_reduced::separate_inner(const NumMatrix& xvals, IloExpr& cut_lhs, IloNum& cut_rhs)
{
	// Update and resolve subproblem 1
	bool is_feasible1 = separate_step1(xvals, cut_lhs, cut_rhs);
	if (!is_feasible1) {
		++flow_cut_count;
		return;
	}

	// Extract y values
	NumMatrix yvals(env, K);
	LOOP(k, K) {
		yvals[k] = NumArray(env, A2);
		subcplex1[k].getValues(y[k], yvals[k]);
	}

	// Update and resolve subproblem 3
	update_subproblem3(xvals, yvals);

	auto sub3start = chrono::high_resolution_clock::now();
	bool is_feasible3 = subcplex3.solve();
	auto sub3end = chrono::high_resolution_clock::now();
	subprob3_time += chrono::duration<double>(sub3end - sub3start).count();

	if (is_feasible3) {
		// Primal feasible, get dual values
		LOOP(k, K) {
			subcplex3.getDuals(f[k], dual_feas[k]);
			subcplex3.getDuals(alpha[k], bilinear1[k]);
			subcplex3.getDuals(beta[k], bilinear2[k]);
		}
		subcplex3.getDuals(rho, equal_obj);
	}
	else {
		// Primal infeasible, get dual Farkas certificate (dual extreme ray)
		update_const_val_map();

		IloConstraintArray const_array(env, 0);
		NumArray dual_vals(env, 0);

		subcplex3.dualFarkas(const_array, dual_vals);

		for (int i = 0; i < const_array.getSize(); i++) {
			IloNum* val_ref = const_val_map[const_array[i].getId()];
			if (val_ref)
				(*val_ref) = -dual_vals[i];
		}

		const_array.end();
		dual_vals.end();
	}

	// Rescale mu
	LOOP(k, K) LOOP(i, V) {
		mu[k][i] *= -rho[k];
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
	if (is_feasible3) {
		cut_lhs.setLinearCoef(v, -1);
		++opt_cut_count;
	}
	else {
		++toll_cut_count;
	}

	// Clean up
	LOOP(k, K) yvals[k].end();
	yvals.end();
}

bool benders_model_reduced::separate_step1(const NumMatrix& xvals, IloExpr& cut_lhs, IloNum& cut_rhs) {
	// Update and resolve subproblem 1
	update_subproblem1(xvals);
	LOOP(k, K) {
		auto sub1start = chrono::high_resolution_clock::now();
		bool is_feasible = subcplex1[k].solve();
		auto sub1end = chrono::high_resolution_clock::now();
		subprob1_time += chrono::duration<double>(sub1end - sub1start).count();

		if (!is_feasible) {
			// FLOW FEASIBILITY CUT
			// Extract dual ray
			update_const_val_map();

			IloConstraintArray const_array(env, 0);
			NumArray dual_vals(env, 0);

			subcplex1[k].dualFarkas(const_array, dual_vals);

			for (int i = 0; i < const_array.getSize(); i++) {
				IloNum* val_ref = const_val_map[const_array[i].getId()];
				if (val_ref)
					(*val_ref) = -dual_vals[i];
			}

			const_array.end();
			dual_vals.end();

			// Cut formulation
			// Right-hand side
			cut_rhs -= mu[k][prob.commodities[k].origin];
			cut_rhs += mu[k][prob.commodities[k].destination];

			// Left-hand side
			LOOP(a, A1) {
				SRC_DST_FROM_A1(prob, a);
				cut_lhs.setLinearCoef(x[k][a], -mu[k][src] + mu[k][dst]);
			}

			return false;
		}
	}

	// All feasible, extract dual values
	LOOP(k, K) {
		// Note that the sign of mu here is flipped
		subcplex1[k].getDuals(mu[k], flow_constr[k]);
	}

	return true;
}

solution benders_model_reduced::get_solution()
{
	NumMatrix xvals = get_values(cplex, x);

	// Resolve subproblem 1 to get y
	update_subproblem1(xvals);

	NumMatrix yvals(env, K);
	LOOP(k, K) {
		subcplex1[k].solve();
		yvals[k] = get_values(subcplex1[k], y[k]);
	}

	// Resolve subproblem 3 to get t
	update_subproblem3(xvals, yvals);
	subcplex3.solve();

	NumArray tvals = get_values(subcplex3, t);

	solution sol = fetch_solution_from_xy_t(*this, xvals, yvals, tvals);

	clean_up(xvals);
	clean_up(yvals);
	tvals.end();

	return sol;
}

std::string benders_model_reduced::get_report()
{
	ostringstream ss;
	ss << "OBJ: " << cplex.getObjValue() << endl <<
		"TIME: " << cplex.getTime() << " s" <<
		"    Sep " << separate_time << " s" <<
		"    Avg " << (separate_time * 1000 / separate_count) << " ms" <<
		"    Sub1 " << (subprob1_time * 100 / separate_time) << "%" <<
		"    Sub3 " << (subprob3_time * 100 / separate_time) << "%" << endl <<
		"SEP: Total " << separate_count <<
		"    F " << flow_cut_count <<
		"    T " << toll_cut_count <<
		"    O " << opt_cut_count << endl;

	return ss.str();
}

IloCplex::Callback benders_model_reduced::attach_callback()
{
	return cplex.use(benders_model_reduced_callback(cplex_model.getEnv(), *this));
}