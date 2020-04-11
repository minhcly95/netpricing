#pragma once

#include "../../model.h"

struct hybrid_model;

struct formulation : public cplex_def {
	hybrid_model* model;
	int k;

	IloEnv env;
	problem* prob;

	IloModel cplex_model;
	IloObjective obj;
	VarArray t;

	int K, V, A, A1, A2;

	void formulate(hybrid_model* model, int k);
	virtual void formulate_impl() = 0;

	virtual bool has_callback() { return false; }
	virtual void invoke_callback(const IloCplex::Callback::Context& context) {}
};
