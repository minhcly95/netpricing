#include "light_vfcut_model.h"

#include "../macros.h"
#include "model_utils.h"

#include <map>
#include <utility>
#include <iostream>
#include <sstream>
#include <chrono>

#include <boost/graph/dijkstra_shortest_paths.hpp>

using namespace std;
using namespace boost;

struct light_vfcut_model_callback : public IloCplex::Callback::Function {
	light_vfcut_model& m;
	double tol;
	model_cplex::NumVarArray heur_vars;

	light_vfcut_model_callback(light_vfcut_model& _m)
		: m(_m), tol(1e-6), heur_vars(m.get_cplex().getEnv()) {

		LOOP(k, m.K) heur_vars.add(m.x[k]);
		LOOP(k, m.K) heur_vars.add(m.y[k]);
		heur_vars.add(m.t);
		LOOP(k, m.K) heur_vars.add(m.tx[k]);
	}

	virtual ~light_vfcut_model_callback() {
		heur_vars.end();
	}

	virtual void invoke(const IloCplex::Callback::Context& context) override {
		using NumArray = light_vfcut_model::NumArray;
		using NumMatrix = light_vfcut_model::NumMatrix;
		using RangeArray = light_vfcut_model::RangeArray;

		IloEnv env = context.getEnv();

		// Only run every 100 nodes
		int node_count = context.getIntInfo(IloCplex::Callback::Context::Info::NodeCount);
		if (node_count % 20 > 0)
			return;

		// Extract t values
		NumArray tvals(env, m.A1);
		context.getRelaxationPoint(m.t, tvals);

		// Separation algorithm
		RangeArray cuts(env);
		NumMatrix xvals(env), yvals(env);
		IloNum obj;

		m.separate(tvals, cuts, xvals, yvals, obj);

		// Add the cuts
		bool post_heur = true;
		LOOP(i, cuts.getSize())
			context.addUserCut(cuts[i], IloCplex::CutManagement::UseCutFilter, false);

		// Post heuristic solution
		if (post_heur && obj > context.getIncumbentObjective()) {
			NumArray heur_vals(env);

			LOOP(k, m.K) heur_vals.add(xvals[k]);
			LOOP(k, m.K) heur_vals.add(yvals[k]);
			heur_vals.add(tvals);
			LOOP(k, m.K) LOOP(a, m.A1) heur_vals.add(tvals[a] * xvals[k][a]);

			context.postHeuristicSolution(heur_vars, heur_vals, obj, IloCplex::Callback::Context::SolutionStrategy::Solve);

			cout << "P";
			heur_vals.end();
		}

		// Clean up
		LOOP(i, cuts.getSize()) cuts[i].end();
		cuts.end();
		clean_up(xvals);
		clean_up(yvals);
		tvals.end();
	}
};

light_vfcut_model::light_vfcut_model(IloEnv& env, const problem& _prob) :
	model_with_generic_callbacks(env), model_single(_prob), lsolver(prob),
	separate_time(0), subprob_time(0), separate_count(0) {
	// Typedef
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
		auto edge = prob.alledges_index_map.left.at(a);
		int src = source(edge, prob.graph);
		int dst = target(edge, prob.graph);

		LOOP(k, K) flow_constr[k][src].setLinearCoef(z[k][a], 1);
		LOOP(k, K) flow_constr[k][dst].setLinearCoef(z[k][a], -1);
	}

	// Dual feasibility
	dual_feas = RangeMatrix(env, K);
	LOOP(k, K) dual_feas[k] = RangeArray(env, A);
	LOOP(a, A) {
		auto edge = prob.alledges_index_map.left.at(a);
		int src = source(edge, prob.graph);
		int dst = target(edge, prob.graph);
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
		auto edge = prob.alledges_index_map.left.at(a);
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

	init_variable_name();
}

void light_vfcut_model::init_variable_name() {
	LOOP(k, K) {
		LOOP(a, A1) {
			SRC_DST_FROM_A1(prob, a);
			char name[50];
			sprintf(name, "x[%d,%d->%d]", k, src, dst);
			x[k][a].setName(name);
		}

		LOOP(a, A2) {
			SRC_DST_FROM_A2(prob, a);
			char name[50];
			sprintf(name, "y[%d,%d->%d]", k, src, dst);
			y[k][a].setName(name);
		}

		LOOP(a, A1) {
			SRC_DST_FROM_A1(prob, a);
			char name[50];
			sprintf(name, "tx[%d,%d->%d]", k, src, dst);
			tx[k][a].setName(name);
		}

		LOOP(i, V) {
			char name[50];
			sprintf(name, "lbd[%d,%d]", k, i);
			lambda[k][i].setName(name);
		}
	}

	LOOP(a, A1) {
		SRC_DST_FROM_A1(prob, a);
		char name[50];
		sprintf(name, "t[%d->%d]", src, dst);
		t[a].setName(name);
	}
}

void light_vfcut_model::separate(const NumArray& tvals,
								RangeArray& cuts, NumMatrix& xvals, NumMatrix& yvals, IloNum& obj) {
	auto start = chrono::high_resolution_clock::now();

	separate_inner(tvals, cuts, xvals, yvals, obj);

	auto end = chrono::high_resolution_clock::now();
	separate_time += chrono::duration<double>(end - start).count();
	++separate_count;
}

void light_vfcut_model::separate_inner(const NumArray& tvals,
									  RangeArray& cuts, NumMatrix& xvals, NumMatrix& yvals, IloNum& obj)
{
	vector<cost_type> tolls(A1);
	LOOP(a, A1) tolls[a] = tvals[a];

	auto substart = chrono::high_resolution_clock::now();
	auto paths = lsolver.solve(tolls);
	auto subend = chrono::high_resolution_clock::now();
	subprob_time += chrono::duration<double>(subend - substart).count();

	obj = 0;

	LOOP(k, K) {
		// Cut formulation
		IloExpr cut_lhs(cplex_model.getEnv());
		IloNum cut_rhs = 0;

		// Solution formulation
		NumArray xk(env, A1), yk(env, A2);
		IloNum objk = 0;

		// Fixed part
		LOOP(a, A) {
			auto edge = A_TO_EDGE(prob, a);
			cost_type cost = prob.cost_map[edge];
			cut_lhs.setLinearCoef(z[k][a], cost);
		}
		LOOP(a, A1) cut_lhs.setLinearCoef(tx[k][a], 1);

		// Path-depending part
		auto& path = paths[k];
		for (int i = 0; i < path.size() - 1; i++) {
			int src = path[i];
			int dst = path[i + 1];
			auto edge = EDGE_FROM_SRC_DST(prob, src, dst);

			cut_rhs += prob.cost_map[edge];
			if (prob.is_tolled_map[edge]) {
				int a1 = EDGE_TO_A1(prob, edge);
				cut_lhs.setLinearCoef(t[a1], -1);

				xk[a1] = 1;
				objk += tvals[a1];
			}
			else {
				int a2 = EDGE_TO_A2(prob, edge);
				yk[a2] = 1;
			}
		}

		cuts.add(cut_lhs <= cut_rhs);

		xvals.add(xk);
		yvals.add(yk);
		obj += objk * prob.commodities[k].demand;
	}
}

solution light_vfcut_model::get_solution()
{
	NumMatrix zvals = get_values(cplex, z);
	NumArray tvals = get_values(cplex, t);

	solution sol = fetch_solution_from_z_t(*this, zvals, tvals);

	clean_up(zvals);
	tvals.end();

	return sol;
}

std::string light_vfcut_model::get_report()
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

vector<pair<IloCplex::Callback::Function*, light_vfcut_model::ContextId>> light_vfcut_model::attach_callbacks()
{
	return {
		make_pair(new light_vfcut_model_callback(*this), CPX_CALLBACKCONTEXT_RELAXATION)
	};
}
