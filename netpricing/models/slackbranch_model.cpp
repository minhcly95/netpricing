#include "slackbranch_model.h"

#include "../macros.h"
#include "model_utils.h"

#include <map>
#include <utility>
#include <sstream>
#include <chrono>

ILOCPLEXGOAL1(slackbranch_model_goal, slackbranch_model&, m) {
	using namespace std;
	using NumArray = slackbranch_model::NumArray;
	using NumMatrix = slackbranch_model::NumMatrix;
	using RangeArray = slackbranch_model::RangeArray;

	auto start = chrono::high_resolution_clock::now();

	IloEnv env(getEnv());
	double tol = m.cplex.getParam(IloCplex::Param::Simplex::Tolerances::Feasibility);

	if (isIntegerFeasible())
		return 0;

	// Slack branching
	NumMatrix xvals(env, m.K);
	IloArray<IntegerFeasibilityArray> feas(env, m.K);

	LOOP(k, m.K) {
		xvals[k] = NumArray(env);
		getValues(xvals[k], m.x[k]);
		feas[k] = IntegerFeasibilityArray(env);
		getFeasibilities(feas[k], m.x[k]);
	}

	int bestk = -1, besta = -1;
	double bestscore = 0;

	LOOP(k, m.K) {
		double demand = m.prob.commodities[k].demand;
		LOOP(a, m.A1) {
			if (feas[k][a] == Infeasible) {
				IloRange dual_con = m.dual_feas[k][A1_TO_A(m.prob, a)];

				double x_inf = xvals[k][a];
				double slack = getSlack(dual_con);
				if (abs(slack) <= tol)
					continue;

				double score = demand * slack * x_inf;

				if (score > bestscore) {
					bestscore = score;
					bestk = k;
					besta = a;
				}
			}
		}
	}

	IloCplex::Goal res;
	if (bestk >= 0) {
		IloNumVar var = m.x[bestk][besta];
		//IloRange comp_range = m.dual_feas[bestk][besta];
		//auto expr = comp_range.getExpr();
		//double rhs = comp_range.getUB();

		res = AndGoal(OrGoal(var == 0, var == 1), this);
	}
	else {
		res = AndGoal(BranchAsCplexGoal(env), this);
	}

	clean_up(xvals);
	LOOP(k, m.K) feas[k].end();
	feas.end();

	auto end = chrono::high_resolution_clock::now();
	m.goal_time += chrono::duration<double>(end - start).count();

	return res;
}

slackbranch_model::slackbranch_model(IloEnv& env, const problem& _prob) :
	model_with_goal(env, slackbranch_model_goal(env, *this)), model_single(_prob), goal_time(0) {
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
		y[k] = NumVarArray(env, A2, 0, IloInfinity);
		z[k] = NumVarArray(env, A);
		lambda[k] = NumVarArray(env, V, -IloInfinity, IloInfinity);
		tx[k] = NumVarArray(env, A1, 0, IloInfinity);
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

	// Equal objective
	equal_obj = RangeArray(env, K, 0, 0);
	LOOP(a, A) {
		auto edge = A_TO_EDGE(prob, a);
		cost_type cost = prob.cost_map[edge];
		LOOP(k, K) equal_obj[k].setLinearCoef(z[k][a], cost);
	}
	LOOP(a, A1) LOOP(k, K) equal_obj[k].setLinearCoef(tx[k][a], 1);
	LOOP(k, K) {
		equal_obj[k].setLinearCoef(lambda[k][prob.commodities[k].origin], -1);
		equal_obj[k].setLinearCoef(lambda[k][prob.commodities[k].destination], 1);
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
	LOOP(k, K) {
		cplex_model.add(flow_constr[k]);
		cplex_model.add(dual_feas[k]);
		cplex_model.add(equal_obj[k]);
		cplex_model.add(bilinear1[k]);
		cplex_model.add(bilinear2[k]);
		cplex_model.add(bilinear3[k]);
	}
}

solution slackbranch_model::get_solution()
{
	NumMatrix zvals = get_values(cplex, z);
	NumArray tvals = get_values(cplex, t);

	solution sol = fetch_solution_from_z_t(*this, zvals, tvals);

	clean_up(zvals);
	tvals.end();

	return sol;
}

std::string slackbranch_model::get_report()
{
	using namespace std;

	ostringstream ss;
	ss << "OBJ: " << cplex.getObjValue() << endl <<
		"TIME: " << get_time() << " s" <<
		"    Goal " << goal_time << " s" << endl;

	return ss.str();
}
