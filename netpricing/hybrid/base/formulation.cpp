#include "formulation.h"

#include "hybrid_model.h"

void formulation::formulate(hybrid_model* model, int k) {
	this->model = model;
	this->k = k;

	env = model->env;
	prob = &model->prob;

	cplex_model = model->cplex_model;
	obj = model->obj;
	t = model->t;

	K = model->K;
	V = model->V;
	A = model->A;
	A1 = model->A1;
	A2 = model->A2;

	formulate_impl();
}
